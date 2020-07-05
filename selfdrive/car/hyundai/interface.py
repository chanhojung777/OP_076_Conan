#!/usr/bin/env python3
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.car.hyundai.values import Ecu, ECU_FINGERPRINT, CAR, FINGERPRINTS, Buttons
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase, MAX_CTRL_SPEED
from selfdrive.atom_conf import AtomConf
from common.params import Params

ATOMC = AtomConf()
params = Params()

EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState )

    self.meg_timer = 0
    self.meg_name = 0
    self.pre_button = 0


  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), has_relay=False, car_fw=[]):
    global ATOMC    
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)

    ret.carName = "hyundai"
    ret.safetyModel = car.CarParams.SafetyModel.hyundai
    #ret.radarOffCan = False  #False(선행차우선)  #True(차선우선)    #선행차량 인식 마크 유무.

    ret.radarOffCan = int( params.get('OpkrTraceSet') ) 
    ret.lateralsRatom.learnerParams = int( params.get('OpkrEnableLearner') ) 
    


    # Hyundai port is a community feature for now
    ret.communityFeature = False  #True

    ret.longcontrolEnabled = False

    """
      0.7.5
      ret.steerActuatorDelay = 0.1  # Default delay   0.1
      ret.steerRateCost = 0.5
      ret.steerLimitTimer = 0.4
      tire_stiffness_factor = 1
    """

    """
      0.7.3
      ret.steerActuatorDelay = 0.10  # Default delay   0.15
      ret.steerRateCost = 0.45
      ret.steerLimitTimer = 0.8
      tire_stiffness_factor = 0.7
    """

    tire_stiffness_factor = 1.
    ret.steerActuatorDelay = 0.09  # Default delay
    ret.steerRateCost = 0.5
    ret.steerLimitTimer = 0.4


    if candidate == CAR.GRANDEUR_HYBRID:
      ret.lateralTuning.pid.kf = 0.000005      
      ret.mass = 1675. + STD_CARGO_KG
      ret.wheelbase = 2.845
      ret.steerRatio = 12.37  #12.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.SANTA_FE:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 3982. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.766
      # Values from optimizer
      ret.steerRatio = 16.55  # 13.8 is spec end-to-end
      tire_stiffness_factor = 0.82
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[9., 22.], [9., 22.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.2, 0.35], [0.05, 0.09]]
    elif candidate == CAR.SONATA:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.SONATA_2019:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
      ret.steerRatio = 13.27 * 1.15   # 15% higher at the center seems reasonable
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.PALISADE:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
      ret.steerRatio = 13.75 * 1.15
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.KIA_SORENTO:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1   # 10% higher at the center seems reasonable
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate in [CAR.ELANTRA, CAR.ELANTRA_GT_I30]:
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 15.4            # 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535
      tire_stiffness_factor = 0.385    # stiffnessFactor settled on 1.0081302973865127
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.HYUNDAI_GENESIS:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.16], [0.01]]
      ret.minSteerSpeed = 60 * CV.KPH_TO_MS
    elif candidate == CAR.GENESIS_G80:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.16], [0.01]]
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2200
      ret.wheelbase = 3.15
      ret.steerRatio = 12.069
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.16], [0.01]]
    elif candidate in [CAR.KIA_OPTIMA, CAR.KIA_OPTIMA_H]:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.KIA_STINGER:
      #ret.lateralTuning.pid.kf = 0.00005
      #ret.mass = 1825. + STD_CARGO_KG
      #ret.wheelbase = 2.78
      #ret.steerRatio = 14.4 * 1.15   # 15% higher at the center seems reasonable
      #ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      #ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]

      #lqr
      #ret.lateralTuning.init('lqr')
      #ret.lateralTuning.lqr.scale = 1500.0
      #ret.lateralTuning.lqr.ki = 0.05
      #ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
      #ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
      #ret.lateralTuning.lqr.c = [1., 0.]
      #ret.lateralTuning.lqr.k = [-110.73572306, 451.22718255]
      #ret.lateralTuning.lqr.l = [0.3233671, 0.3185757]
      #ret.lateralTuning.lqr.dcGain = 0.002237852961363602

      # indi
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGain = 3.0
      ret.lateralTuning.indi.outerLoopGain = 2.0
      ret.lateralTuning.indi.timeConstant = 1.0
      ret.lateralTuning.indi.actuatorEffectiveness = 1.5
      ret.steerActuatorDelay = 0.08 # Stinger GT Limited AWD 3.3T stock value (Tunder's 2020) 
      ret.steerLimitTimer = 0.4 # stock is 0.01 but 0.04 seems to work well
      tire_stiffness_factor = 1.125 # LiveParameters (Tunder's 2020)
      ret.steerRateCost = 1.0
      ret.mass = 1825.0 + STD_CARGO_KG
      ret.wheelbase = 2.906 # https://www.kia.com/us/en/stinger/specs
      ret.steerRatio = 13.56   # 10.28 measured by wheel alignment machine/reported steering angle by OP. 2020 GT Limited AWD has a variable steering ratio ultimately ending in 10.28.  13.56 after 1200km in LiveParamaters (Tunder)
    elif candidate == CAR.KONA:
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.KONA_EV:
      ret.lateralTuning.pid.kf = 0.00006
      ret.mass = 1685. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73  # Spec
      tire_stiffness_factor = 0.385
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]
    elif candidate == CAR.KIA_FORTE:
      ret.lateralTuning.pid.kf = 0.00005
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75
      tire_stiffness_factor = 0.5
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.25], [0.05]]



    # 3번 atom param.
    ret.lateralPIDatom.kBPV = [0.15, 0.20]  # 속도.

    ret.steerRatio = ATOMC.steerRatio  #10.5  #12.5
    ret.steerRateCost = ATOMC.steerRateCost #0.4 #0.4
  
    ret.lateralPIDatom.sRkBPV = ATOMC.sR_BPV   # 조향각.
    ret.lateralPIDatom.sRBoostV = ATOMC.sR_BoostV
    ret.lateralPIDatom.sRkpV1 = ATOMC.sR_kpV1
    ret.lateralPIDatom.sRkiV1 = ATOMC.sR_kiV1
    ret.lateralPIDatom.sRkdV1 = ATOMC.sR_kdV1
    ret.lateralPIDatom.sRkfV1 = ATOMC.sR_kfV1

    ret.lateralPIDatom.sRkpV2 = ATOMC.sR_kpV2
    ret.lateralPIDatom.sRkiV2 = ATOMC.sR_kiV2
    ret.lateralPIDatom.sRkdV2 = ATOMC.sR_kdV2
    ret.lateralPIDatom.sRkfV2 = ATOMC.sR_kfV2

    
    ret.lateralCVatom.cvBPV = ATOMC.cvBPV
    ret.lateralCVatom.cvSteerMaxV1 = ATOMC.cvSteerMaxV1
    ret.lateralCVatom.cvSteerDeltaUpV1 = ATOMC.cvSteerDeltaUpV1
    ret.lateralCVatom.cvSteerDeltaDnV1 = ATOMC.cvSteerDeltaDnV1
    ret.lateralCVatom.cvSteerMaxV2 = ATOMC.cvSteerMaxV2
    ret.lateralCVatom.cvSteerDeltaUpV2 = ATOMC.cvSteerDeltaUpV2
    ret.lateralCVatom.cvSteerDeltaDnV2 = ATOMC.cvSteerDeltaDnV2

    ret.lateralsRatom.learnerParams = ATOMC.learnerParams
    ret.lateralsRatom.deadzone = ATOMC.deadzone
    ret.lateralsRatom.steerOffset = ATOMC.steerOffset
    ret.lateralsRatom.tireStiffnessFactor = ATOMC.tire_stiffness_factor
    tire_stiffness_factor = ATOMC.tire_stiffness_factor


    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, Ecu.fwdCamera) or has_relay

    return ret

  @staticmethod
  def live_tune(CP, read=False):
    global ATOMC 

    if read:
      ATOMC.read_tune()

    CP.steerRatio = ATOMC.steerRatio  #10.5  #12.5
    CP.steerRateCost = ATOMC.steerRateCost #0.4 #0.4
  
    CP.lateralPIDatom.sRkBPV = ATOMC.sR_BPV   # 조향각.
    CP.lateralPIDatom.sRBoostV = ATOMC.sR_BoostV
    CP.lateralPIDatom.sRkpV1 = ATOMC.sR_kpV1
    CP.lateralPIDatom.sRkiV1 = ATOMC.sR_kiV1
    CP.lateralPIDatom.sRkdV1 = ATOMC.sR_kdV1
    CP.lateralPIDatom.sRkfV1 = ATOMC.sR_kfV1

    CP.lateralPIDatom.sRkpV2 = ATOMC.sR_kpV2
    CP.lateralPIDatom.sRkiV2 = ATOMC.sR_kiV2
    CP.lateralPIDatom.sRkdV2 = ATOMC.sR_kdV2
    CP.lateralPIDatom.sRkfV2 = ATOMC.sR_kfV2
    
    CP.lateralCVatom.cvBPV = ATOMC.cvBPV
    CP.lateralCVatom.cvSteerMaxV1 = ATOMC.cvSteerMaxV1
    CP.lateralCVatom.cvSteerDeltaUpV1 = ATOMC.cvSteerDeltaUpV1
    CP.lateralCVatom.cvSteerDeltaDnV1 = ATOMC.cvSteerDeltaDnV1
    CP.lateralCVatom.cvSteerMaxV2 = ATOMC.cvSteerMaxV2
    CP.lateralCVatom.cvSteerDeltaUpV2 = ATOMC.cvSteerDeltaUpV2
    CP.lateralCVatom.cvSteerDeltaDnV2 = ATOMC.cvSteerDeltaDnV2

    #CP.lateralsRatom.learnerParams = ATOMC.learnerParams
    CP.lateralsRatom.deadzone = ATOMC.deadzone
    CP.lateralsRatom.steerOffset = ATOMC.steerOffset
    CP.lateralsRatom.tireStiffnessFactor = ATOMC.tire_stiffness_factor    
    return CP

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid


    # TODO: button presses
    buttonEvents = []
    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.unknown
      if self.CS.cruise_buttons != 0:
        be.pressed = True
        but = self.CS.cruise_buttons
      else:
        be.pressed = False
        but = self.CS.prev_cruise_buttons
      if but == Buttons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == Buttons.SET_DECEL:
        be.type = ButtonType.decelCruise
      elif but == Buttons.CANCEL:
        be.type = ButtonType.cancel
      buttonEvents.append(be)
    if self.CS.cruise_main_button != self.CS.prev_cruise_main_button:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton3
      be.pressed = bool(self.CS.cruise_main_button)
      buttonEvents.append(be)
    ret.buttonEvents = buttonEvents
    #ret.buttonEvents = []    

    events = self.create_common_events(ret)


    
    if not self.cruise_enabled_prev:
      self.meg_timer = 0
      self.meg_name =  None
    else:
      meg_timer = 100
      if self.meg_timer:
        self.meg_timer -= 1
        meg_timer = 0
      elif not self.CS.lkas_button_on:
        self.meg_name = EventName.invalidLkasSetting
      elif ret.cruiseState.standstill:
        self.meg_name = EventName.resumeRequired       
      elif self.CC.lane_change_torque_lower:
        self.meg_name = EventName.laneChangeManual
      elif self.CC.steer_torque_over_timer and self.CC.steer_torque_ratio < 0.1:
        self.meg_name = EventName.steerTorqueOver
      elif self.CC.steer_torque_ratio < 0.5 and self.CS.clu_Vanz > 5:
        self.meg_name = EventName.steerTorqueLow
      elif ret.vEgo > MAX_CTRL_SPEED:
        self.meg_name = EventName.speedTooHigh
      elif ret.steerError:
        self.meg_name = EventName.steerUnavailable
      elif ret.steerWarning:
        self.meg_name = EventName.steerTempUnavailable
      else:
        meg_timer = 0
        self.meg_name =  None

      if meg_timer != 0:
        self.meg_timer = 100

      if self.meg_timer and  self.meg_name != None:
        events.add( self.meg_name )
    

    #TODO: addd abs(self.CS.angle_steers) > 90 to 'steerTempUnavailable' event

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + 2.) and self.CP.minSteerSpeed > 10.:
      self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + 4.):
      self.low_speed_alert = False
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c, sm, CP ):
    can_sends = self.CC.update(c, self.CS, self.frame, sm, CP )

    self.frame += 1
    return can_sends
