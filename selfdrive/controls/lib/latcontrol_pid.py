from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from cereal import car
from cereal import log

from selfdrive.car.hyundai.interface import CarInterface
from common.numpy_fast import interp
import common.log as trace1

ButtonType = car.CarState.ButtonEvent.Type

class LatControlPID():
  def __init__(self, CP):
    self.trPID = trace1.Loger("pid")
    self.angle_steers_des = 0.
    self.prev_cruise_buttons = 0

    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0, sat_limit=CP.steerLimitTimer)



  def reset(self):
    self.pid.reset()



  def linear2_tune( self, CS, CP ):  # angle(조향각에 의한 변화)
    v_ego = CS.vEgo

    
    if self.prev_cruise_buttons != CS.cruiseState.enabled:
      self.prev_cruise_buttons = CS.cruiseState.enabled
      if self.prev_cruise_buttons:
        CP = CarInterface.live_tune( CP )

    self.kBPV = CP.lateralPIDatom.kBPV
    self.sRkBPV = CP.lateralPIDatom.sRkBPV
    self.sRBoostV = CP.lateralPIDatom.sRBoostV

    self.sRkpV1 = CP.lateralPIDatom.sRkpV1
    self.sRkiV1 = CP.lateralPIDatom.sRkiV1
    self.sRkdV1 = CP.lateralPIDatom.sRkdV1
    self.sRkfV1 = CP.lateralPIDatom.sRkfV1
    self.sRkpV2 = CP.lateralPIDatom.sRkpV2
    self.sRkiV2 = CP.lateralPIDatom.sRkiV2
    self.sRkdV2 = CP.lateralPIDatom.sRkdV2
    self.sRkfV2 = CP.lateralPIDatom.sRkfV2

    self.deadzone =  CP.lateralsRatom.deadzone


    str1 = 'bp={}  srBP={} sRBoost={} sRkpV={},{} sRkiV={},{} sRkdV={},{} sRkfV={},{}'.format( self.kBPV, self.sRkBPV, self.sRBoostV, self.sRkpV1, self.sRkpV2, self.sRkiV1, self.sRkiV2, self.sRkdV1, self.sRkdV2, self.sRkfV1, self.sRkfV2 )
    self.trPID.add( str1 )    

    cv_angle = abs(self.angle_steers_des)
    cv = self.sRkBPV   # angle
    # Kp
    fKp1 = self.sRkpV1
    fKp2 = self.sRkpV2
    self.steerKp1 = interp( cv_angle, cv, fKp1 )
    self.steerKp2 = interp( cv_angle, cv, fKp2 )
    self.steerKpV = [ float(self.steerKp1), float(self.steerKp2) ]

    # Ki
    fKi1 = self.sRkiV1
    fKi2 = self.sRkiV2
    self.steerKi1 = interp( cv_angle, cv, fKi1 )
    self.steerKi2 = interp( cv_angle, cv, fKi2 )
    self.steerKiV = [ float(self.steerKi1), float(self.steerKi2) ]

    # Kd
    fKd1 = self.sRkdV1
    fKd2 = self.sRkdV2
    self.steerKd1 = interp( cv_angle, cv, fKd1 )
    self.steerKd2 = interp( cv_angle, cv, fKd2 )    
    self.steerKdV = [ float(self.steerKd1), float(self.steerKd2) ]

    # kf
    fKf1 = self.sRkfV1
    fKf2 = self.sRkfV2
    self.steerKf1 = interp( cv_angle, cv, fKf1 )
    self.steerKf2 = interp( cv_angle, cv, fKf2 )

    kBP = self.kBPV
    fp = [float(self.steerKf1), float(self.steerKf2) ]
    self.steerKf = interp( v_ego,  kBP, fp )

    self.pid.gain( (kBP, self.steerKpV), (kBP, self.steerKiV), k_f=self.steerKf, k_d=(kBP, self.steerKdV) )



  def update(self, active, CS, CP, path_plan):
    self.angle_steers_des = path_plan.angleSteers    
    self.linear2_tune( CS, CP )

    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steerAngle = float(CS.steeringAngle)
    pid_log.steerRate = float(CS.steeringRate)


    if CS.vEgo < 0.3 or not active:
      output_steer = 0.0
      pid_log.active = False
      self.reset()
    else:
      steers_max = get_steer_max(CP, CS.vEgo)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      steer_feedforward = self.angle_steers_des   # feedforward desired angle
      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # TODO: feedforward something based on path_plan.rateSteers
        steer_feedforward -= path_plan.angleOffset   # subtract the offset, since it does not contribute to resistive torque
        steer_feedforward *= CS.vEgo**2  # proportional to realigning tire momentum (~ lateral accel)
      deadzone = self.deadzone  #0.1

      check_saturation = (CS.vEgo > 10) and not CS.steeringRateLimited and not CS.steeringPressed
      output_steer = self.pid.update(self.angle_steers_des, CS.steeringAngle, check_saturation=check_saturation, override=CS.steeringPressed,
                                     feedforward=steer_feedforward, speed=CS.vEgo, deadzone=deadzone)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)

    return output_steer, float(self.angle_steers_des), pid_log
