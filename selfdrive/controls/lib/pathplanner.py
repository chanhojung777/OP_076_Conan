import os
import math
from common.realtime import sec_since_boot, DT_MDL
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc import libmpc_py
from selfdrive.controls.lib.drive_helpers import MPC_COST_LAT
from selfdrive.controls.lib.lane_planner import LanePlanner
from selfdrive.config import Conversions as CV
from selfdrive.car.hyundai.carstate import ATOMC
from common.params import Params
from common.numpy_fast import interp
import cereal.messaging as messaging
from cereal import log

import common.log as trace1




LaneChangeState = log.PathPlan.LaneChangeState
LaneChangeDirection = log.PathPlan.LaneChangeDirection

LOG_MPC = os.environ.get('LOG_MPC', False)

LANE_CHANGE_SPEED_MIN = 45 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.PathPlan.Desire.none,
    LaneChangeState.preLaneChange: log.PathPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.PathPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.PathPlan.Desire.laneChangeRight,
  },
}


def calc_states_after_delay(states, v_ego, steer_angle, curvature_factor, steer_ratio, delay):
  states[0].x = v_ego * delay
  states[0].psi = v_ego * curvature_factor * math.radians(steer_angle) / steer_ratio * delay
  return states


class PathPlanner():
  def __init__(self, CP):
    self.LP = LanePlanner()

    self.last_cloudlog_t = 0
    self.steer_rate_cost = CP.steerRateCost

    self.setup_mpc()
    self.solution_invalid_cnt = 0
    self.lane_change_enabled = Params().get('LaneChangeEnabled') == b'1'
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.prev_one_blinker = False

    # atom
    self.trPATH = trace1.Loger("path")
    self.trLearner = trace1.Loger("Learner")

    self.atom_steer_ratio = None
    self.atom_sr_boost_bp = [0., 0.]
    self.atom_sr_boost_range = [0., 0.]

  def limit_ctrl(self, value, limit, offset ):
      p_limit = offset + limit
      m_limit = offset - limit
      if value > p_limit:
          value = p_limit
      elif  value < m_limit:
          value = m_limit
      return value    

  def setup_mpc(self):
    self.libmpc = libmpc_py.libmpc
    self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steer_rate_cost)

    self.mpc_solution = libmpc_py.ffi.new("log_t *")
    self.cur_state = libmpc_py.ffi.new("state_t *")
    self.cur_state[0].x = 0.0
    self.cur_state[0].y = 0.0
    self.cur_state[0].psi = 0.0
    self.cur_state[0].delta = 0.0

    self.angle_steers_des = 0.0
    self.angle_steers_des_mpc = 0.0
    self.angle_steers_des_prev = 0.0
    self.angle_steers_des_time = 0.0

  def update(self, sm, pm, CP, VM):
    global ATOMC
  
    v_ego = sm['carState'].vEgo
    angle_steers = sm['carState'].steeringAngle
    active = sm['controlsState'].active
    v_ego_kph = v_ego * CV.MS_TO_KPH
    
    self.atom_steer_ratio = sm['liveParameters'].steerRatio
    angle_offset = sm['liveParameters'].angleOffset
    angleOffsetAverage = sm['liveParameters'].angleOffsetAverage
    stiffnessFactor = sm['liveParameters'].stiffnessFactor

    str_log3 = 'angleOffset={:.3f} angleOffsetAverage={:.3f} steerRatio={:.3f} stiffnessFactor={:.5f} '.format( angle_offset, angleOffsetAverage, self.atom_steer_ratio, stiffnessFactor )
    self.trLearner.add( 'LearnerParam {}'.format( str_log3 ) )       
    if ATOMC.LearnerParams:
      pass
    else:
      angle_offset = 0
      angleOffsetAverage = 0
      stiffnessFactor = ATOMC.tire_stiffness_factor
      # atom
      self.atom_sr_boost_bp = ATOMC.sr_boost_bp
      self.atom_sr_boost_range = ATOMC.sr_boost_range
      boost_rate = interp(abs(angle_steers), self.atom_sr_boost_bp, self.atom_sr_boost_range)
      self.atom_steer_ratio = ATOMC.steerRatio + boost_rate
      self.steer_rate_cost = ATOMC.steerRateCost

      str_log1 = 'steerRatio={:.1f}/{:.1f} bp={} range={}'.format(  CP.steerRatio, ATOMC.steerRatio,  self.atom_sr_boost_bp, self.atom_sr_boost_range )
      str_log2 = 'angle_steers={:.0f} steerRatio={:.3f} steerRateCost={:.2f}'.format( angle_steers, self.atom_steer_ratio, self.steer_rate_cost )
      self.trPATH.add( '{} {}'.format( str_log1, str_log2 ) )
      

    # Run MPC
    self.angle_steers_des_prev = self.angle_steers_des_mpc
    VM.update_params(sm['liveParameters'].stiffnessFactor, self.atom_steer_ratio ) # sm['liveParameters'].steerRatio)
    curvature_factor = VM.curvature_factor(v_ego)



    self.LP.parse_model(sm['model'])

    # Lane change logic
    one_blinker = sm['carState'].leftBlinker != sm['carState'].rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if sm['carState'].leftBlinker:
      self.lane_change_direction = LaneChangeDirection.left
    elif sm['carState'].rightBlinker:
      self.lane_change_direction = LaneChangeDirection.right

    if (not active) or (self.lane_change_timer > LANE_CHANGE_TIME_MAX) or (not one_blinker) or (not self.lane_change_enabled):
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      torque_applied = sm['carState'].steeringPressed and \
                       ((sm['carState'].steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or \
                        (sm['carState'].steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

      lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob

      # State transitions
      # off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0

      # pre
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
        elif torque_applied:
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # starting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2*DT_MDL, 0.0)
        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # finishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
        if one_blinker and self.lane_change_ll_prob > 0.99:
          self.lane_change_state = LaneChangeState.preLaneChange
        elif self.lane_change_ll_prob > 0.99:
          self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in [LaneChangeState.off, LaneChangeState.preLaneChange]:
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Turn off lanes during lane change
    if desire == log.PathPlan.Desire.laneChangeRight or desire == log.PathPlan.Desire.laneChangeLeft:
      self.LP.l_prob *= self.lane_change_ll_prob
      self.LP.r_prob *= self.lane_change_ll_prob
    self.LP.update_d_poly(v_ego)

    # account for actuation delay
    self.cur_state = calc_states_after_delay(self.cur_state, v_ego, angle_steers - angle_offset, curvature_factor, VM.sR, CP.steerActuatorDelay)

    v_ego_mpc = max(v_ego, 5.0)  # avoid mpc roughness due to low speed
    self.libmpc.run_mpc(self.cur_state, self.mpc_solution,
                        list(self.LP.l_poly), list(self.LP.r_poly), list(self.LP.d_poly),
                        self.LP.l_prob, self.LP.r_prob, curvature_factor, v_ego_mpc, self.LP.lane_width)

    # reset to current steer angle if not active or overriding
    if active:
      delta_desired = self.mpc_solution[0].delta[1]
      rate_desired = math.degrees(self.mpc_solution[0].rate[0] * VM.sR)
    else:
      delta_desired = math.radians(angle_steers - angle_offset) / VM.sR
      rate_desired = 0.0

    self.cur_state[0].delta = delta_desired

    self.angle_steers_des_mpc = float(math.degrees(delta_desired * VM.sR) + angle_offset)
    org_angle_steers_des = self.angle_steers_des_mpc

    # atom
    if v_ego_kph < 30:
        xp = [5,15,30]
        fp2 = [1,5,7]
        limit_steers = interp( v_ego_kph, xp, fp2 )
        self.angle_steers_des_mpc = self.limit_ctrl( org_angle_steers_des, limit_steers, angle_steers )

    #  Check for infeasable MPC solution
    mpc_nans = any(math.isnan(x) for x in self.mpc_solution[0].delta)
    t = sec_since_boot()
    if mpc_nans:
      self.libmpc.init(MPC_COST_LAT.PATH, MPC_COST_LAT.LANE, MPC_COST_LAT.HEADING, self.steer_rate_cost)
      self.cur_state[0].delta = math.radians(angle_steers - angle_offset) / VM.sR

      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.mpc_solution[0].cost > 20000. or mpc_nans:   # TODO: find a better way to detect when MPC did not converge
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0
    plan_solution_valid = self.solution_invalid_cnt < 2

    plan_send = messaging.new_message('pathPlan')
    plan_send.valid = sm.all_alive_and_valid(service_list=['carState', 'controlsState', 'liveParameters', 'model'])
    plan_send.pathPlan.laneWidth = float(self.LP.lane_width)
    plan_send.pathPlan.dPoly = [float(x) for x in self.LP.d_poly]
    plan_send.pathPlan.lPoly = [float(x) for x in self.LP.l_poly]
    plan_send.pathPlan.lProb = float(self.LP.l_prob)
    plan_send.pathPlan.rPoly = [float(x) for x in self.LP.r_poly]
    plan_send.pathPlan.rProb = float(self.LP.r_prob)

    plan_send.pathPlan.angleSteers = float(self.angle_steers_des_mpc)
    plan_send.pathPlan.rateSteers = float(rate_desired)
    plan_send.pathPlan.angleOffset = float(sm['liveParameters'].angleOffsetAverage)
    plan_send.pathPlan.mpcSolutionValid = bool(plan_solution_valid)
    plan_send.pathPlan.paramsValid = bool(sm['liveParameters'].valid)

    plan_send.pathPlan.desire = desire
    plan_send.pathPlan.laneChangeState = self.lane_change_state
    plan_send.pathPlan.laneChangeDirection = self.lane_change_direction

    pm.send('pathPlan', plan_send)

    if LOG_MPC:
      dat = messaging.new_message('liveMpc')
      dat.liveMpc.x = list(self.mpc_solution[0].x)
      dat.liveMpc.y = list(self.mpc_solution[0].y)
      dat.liveMpc.psi = list(self.mpc_solution[0].psi)
      dat.liveMpc.delta = list(self.mpc_solution[0].delta)
      dat.liveMpc.cost = self.mpc_solution[0].cost
      pm.send('liveMpc', dat)
