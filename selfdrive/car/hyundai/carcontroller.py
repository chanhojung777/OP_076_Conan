from cereal import car, log
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfa_mfa, create_mdps12
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from common.numpy_fast import interp
from selfdrive.car.hyundai.spdcontroller  import SpdController
#from selfdrive.car.hyundai.interface import CarInterface
import common.log as trace1
import common.CTime1000 as tm

VisualAlert = car.CarControl.HUDControl.VisualAlert
LaneChangeState = log.PathPlan.LaneChangeState




class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.steer_rate_limited = False
    self.resume_cnt = 0
    self.lkas11_cnt = 0
    self.last_resume_frame = 0
    self.last_lead_distance = 0



    self.nBlinker = 0
    self.lane_change_torque_lower = 0
    self.steer_torque_over_timer = 0
    self.steer_torque_ratio = 1
    self.steer_torque_ratio_dir = 1

    self.dRel = 0
    self.yRel = 0
    self.vRel = 0

    self.timer1 = tm.CTime1000("time")
    self.SC = SpdController()    
    self.model_speed = 0
    self.model_sum = 0
    
    # hud
    self.hud_timer_left = 0
    self.hud_timer_right = 0



  def limit_ctrl(self, value, limit, offset ):
      p_limit = offset + limit
      m_limit = offset - limit
      if value > p_limit:
          value = p_limit
      elif  value < m_limit:
          value = m_limit
      return value


  def process_hud_alert(self, enabled, CC ):
    visual_alert = CC.hudControl.visualAlert
    left_lane = CC.hudControl.leftLaneVisible
    right_lane = CC.hudControl.rightLaneVisible

    sys_warning = (visual_alert == VisualAlert.steerRequired)

    if left_lane:
      self.hud_timer_left = 100

    if right_lane:
      self.hud_timer_right = 100

    if self.hud_timer_left:
      self.hud_timer_left -= 1
 
    if self.hud_timer_right:
      self.hud_timer_right -= 1


    # initialize to no line visible
    sys_state = 1
    if self.hud_timer_left and self.hud_timer_right or sys_warning:  # HUD alert only display when LKAS status is active
      if (self.steer_torque_ratio > 0.7) and (enabled or sys_warning):
        sys_state = 3
      else:
        sys_state = 4
    elif self.hud_timer_left:
      sys_state = 5
    elif self.hud_timer_right:
      sys_state = 6

    return sys_warning, sys_state


  def cV_tune( self, v_ego_kph, cv_value ):  # cV(곡률에 의한 변화)
    #cv_value = abs(self.angle_steers_des)

    self.kBPV = self.CP.lateralPIDatom.kBPV
    self.cVBPV = self.CP.lateralCVatom.cvBPV
    self.cvSteerMaxV1  = self.CP.lateralCVatom.cvSteerMaxV1
    self.cvSteerDeltaUpV1 = self.CP.lateralCVatom.cvSteerDeltaUpV1
    self.cvSteerDeltaDnV1 = self.CP.lateralCVatom.cvSteerDeltaDnV1
    self.cvSteerMaxV2 = self.CP.lateralCVatom.cvSteerMaxV2
    self.cvSteerDeltaUpV2 = self.CP.lateralCVatom.cvSteerDeltaUpV2
    self.cvSteerDeltaDnV2 = self.CP.lateralCVatom.cvSteerDeltaDnV2        

    cv_BPV = self.cVBPV   # 곡률
    # Max
    self.steerMax1 = interp( cv_value, cv_BPV, self.cvSteerMaxV1 )
    self.steerMax2 = interp( cv_value, cv_BPV, self.cvSteerMaxV2 )
    self.steerMaxV = [ float(self.steerMax1), float(self.steerMax2) ]
    self.MAX = interp( v_ego_kph, self.kBPV, self.steerMaxV )  

    # Up
    self.steerUP1 = interp( cv_value, cv_BPV, self.cvSteerDeltaUpV1 )
    self.steerUP2 = interp( cv_value, cv_BPV, self.cvSteerDeltaUpV2 )
    self.steerUPV = [ float(self.steerUP1), float(self.steerUP2) ]
    self.UP = interp( v_ego_kph, self.kBPV, self.steerUPV )

    # dn
    self.steerDN1 = interp( cv_value, cv_BPV, self.cvSteerDeltaDnV1 )
    self.steerDN2 = interp( cv_value, cv_BPV, self.cvSteerDeltaDnV2 )    
    self.steerDNV = [ float(self.steerDN1), float(self.steerDN2) ]
    self.DN = interp( v_ego_kph, self.kBPV, self.steerDNV )



  def steerParams_torque(self, CS, abs_angle_steers, path_plan, CC ):

    param = SteerLimitParams()
    v_ego_kph = CS.out.vEgo * CV.MS_TO_KPH

    self.cV_tune( v_ego_kph, self.model_speed )
    param.STEER_MAX = min( param.STEER_MAX, self.MAX)
    param.STEER_DELTA_UP = min( param.STEER_DELTA_UP, self.UP)
    param.STEER_DELTA_DOWN = min( param.STEER_DELTA_DOWN, self.DN )


    # streer over check
    if v_ego_kph > 5 and abs( CS.out.steeringTorque ) > 180:  #사용자 핸들 토크
      self.steer_torque_over_timer = 1
    else:
      self.steer_torque_over_timer = 0


    if CS.out.leftBlinker or CS.out.rightBlinker:
      self.nBlinker += 1
    elif self.nBlinker:
      self.nBlinker = 0

    # 차선이 없고 앞차량이 없으면.
    steer_angle_lower = self.dRel > 20 and (not CC.hudControl.leftLaneVisible  and not CC.hudControl.rightLaneVisible)

    if v_ego_kph < 1:
      self.steer_torque_over_timer = 0
      self.steer_torque_ratio_dir = 1
    elif path_plan.laneChangeState != LaneChangeState.off:
      self.steer_torque_ratio_dir = 1
      self.steer_torque_over_timer = 0
      self.nBlinker = 0
    elif self.steer_torque_over_timer:  #or CS.out.steerWarning:
      self.steer_torque_ratio_dir = -1
    elif steer_angle_lower:
      param.STEER_MAX *= 0.5
      param.STEER_DELTA_UP  = 1
      param.STEER_DELTA_DOWN = 2
      self.steer_torque_ratio_dir = 1      
    else:
      self.steer_torque_ratio_dir = 1

    lane_change_torque_lower = 0
    if self.nBlinker > 10:
      lane_change_torque_lower = int(CS.out.leftBlinker) + int(CS.out.rightBlinker) * 2
      #self.steer_torque_ratio_dir = 1
      if CS.out.steeringPressed:
        self.steer_torque_ratio = 0.05      

    self.lane_change_torque_lower =  lane_change_torque_lower
    # smoth torque enable or disable
    if self.steer_torque_ratio_dir >= 1:
      if self.steer_torque_ratio < 1:
        self.steer_torque_ratio += 0.002  # 5 sec
    elif self.steer_torque_ratio_dir <= -1:
      if self.steer_torque_ratio > 0:
        self.steer_torque_ratio -= 0.005  # 2 sec

    if self.steer_torque_ratio < 0:
      self.steer_torque_ratio = 0
    elif self.steer_torque_ratio > 1:
      self.steer_torque_ratio = 1

    return  param



  def update(self, CC, CS, frame, sm, CP ):
    if self.CP != CP:
      self.CP = CP
    #self.CP = CarInterface.live_tune( self.CP, False )

    enabled = CC.enabled
    actuators = CC.actuators
    pcm_cancel_cmd = CC.cruiseControl.cancel


    path_plan = sm['pathPlan']

    abs_angle_steers =  abs(actuators.steerAngle)

    self.dRel, self.yRel, self.vRel = SpdController.get_lead( sm )
    self.model_speed, self.model_sum = self.SC.calc_va(  sm, CS.out.vEgo  )


    # Steering Torque
    #param = SteerLimitParams()
    param = self.steerParams_torque( CS, abs_angle_steers, path_plan, CC )


    new_steer = actuators.steer * param.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, param)
    self.steer_rate_limited = new_steer != apply_steer

    apply_steer_limit = param.STEER_MAX
    if self.steer_torque_ratio < 1:
      apply_steer_limit = int(self.steer_torque_ratio * param.STEER_MAX)
      apply_steer = self.limit_ctrl( apply_steer, apply_steer_limit, 0 )


    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    lkas_active = enabled and abs(CS.out.steeringAngle) < 90. #and self.lkas_button

    # fix for Genesis hard fault at low speed
    if CS.out.vEgo < 16.7 and self.car_fingerprint == CAR.HYUNDAI_GENESIS:
      lkas_active = 0

    if not lkas_active:
      apply_steer = 0

    steer_req = 1 if apply_steer else 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state = self.process_hud_alert( lkas_active, CC )

    can_sends = []
    if frame == 0: # initialize counts from last received count signals
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"] + 1
    self.lkas11_cnt %= 0x10

    can_sends.append(create_lkas11(self.packer, self.lkas11_cnt, self.car_fingerprint, apply_steer, steer_req,
                                   CS.lkas11, sys_warning, sys_state, CC ))

    can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))

    str_log1 = 'torg:{:5.0f} C={:.1f}/{:.1f} V={:.1f}/{:.1f} CV={:.1f}/{:.3f}'.format(  apply_steer, CS.lead_objspd, CS.lead_distance, self.dRel, self.vRel, self.model_speed, self.model_sum )
    str_log2 = 'limit={:.0f} LC={} tm={:.1f}'.format( apply_steer_limit, path_plan.laneChangeState, self.timer1.sampleTime()  )
    trace1.printf( '{} {}'.format( str_log1, str_log2 ) )

    str_log2 = 'U={:.0f}  LK={:.0f} dir={} steer={:5.0f} '.format( CS.Mdps_ToiUnavail, CS.lkas_button_on, self.steer_torque_ratio_dir, CS.out.steeringTorque  )
    trace1.printf2( '{}'.format( str_log2 ) )

    if pcm_cancel_cmd:
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL))

    elif CS.out.cruiseState.standstill:
      # run only first time when the car stopped
      if self.last_lead_distance == 0:
        # get the lead distance from the Radar
        self.last_lead_distance = CS.lead_distance
        self.resume_cnt = 0
      # when lead car starts moving, create 6 RES msgs
      elif CS.lead_distance != self.last_lead_distance and (frame - self.last_resume_frame) > 5:
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL))
        self.resume_cnt += 1
        # interval after 6 msgs
        if self.resume_cnt > 5:
          self.last_resume_frame = frame
    # reset lead distnce after the car starts moving
    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0


    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in [CAR.SONATA, CAR.PALISADE]:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    # counter inc
    self.lkas11_cnt += 1
    return can_sends

