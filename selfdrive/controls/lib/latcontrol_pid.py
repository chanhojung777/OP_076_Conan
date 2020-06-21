from selfdrive.controls.lib.pid import PIController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from cereal import car
from cereal import log

from common.numpy_fast import interp
from selfdrive.car.hyundai.carstate import ATOMC
import common.log as trace1

ButtonType = car.CarState.ButtonEvent.Type

class LatControlPID():
  def __init__(self, CP):
    self.trPID = trace1.Loger("pid")   
    self.angle_steers_des = 0.


    self.pid = PIController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0, sat_limit=CP.steerLimitTimer)

  def reset(self):
    self.pid.reset()



  def linear2_tune( self, CP, v_ego ):  # angle(조향각에 의한 변화)
    global ATOMC

    self.steer_Kp1 = ATOMC.steer_Kp1  #[0.11,0.12]
    self.steer_Ki1 = ATOMC.steer_Ki1  #[0.008,0.01]
    self.steer_Kd1 = ATOMC.steer_Kd1
    self.steer_Kf1 = ATOMC.steer_Kf1 #[0.000001,0.00001]

    self.steer_Kp2 = ATOMC.steer_Kp2 #[0.13,0.15]
    self.steer_Ki2 = ATOMC.steer_Ki2 #[0.015,0.02]
    self.steer_Kd2 = ATOMC.steer_Kd2 #[0.015,0.02]
    self.steer_Kf2 = ATOMC.steer_Kf2  #[0.00003,0.00003]

    self.sr_boost_bp = ATOMC.sr_boost_bp
    self.deadzone = ATOMC.deadzone


    str1 = 'bp={}  kp={},{} ki={},{} kd={},{} kf={},{}'.format( self.sr_boost_bp, self.steer_Kp1, self.steer_Kp2, self.steer_Ki1, self.steer_Ki2, self.steer_Kd1, self.steer_Kd2, self.steer_Kf1, self.steer_Kf2 )
    self.trPID.add( str1 )    

    cv_angle = abs(self.angle_steers_des)
    cv = self.sr_boost_bp  #[ 4, 30 ]  # angle
    # Kp
    fKp1 = [float(self.steer_Kp1[ 0 ]), float(self.steer_Kp1[ 1 ]) ]
    fKp2 = [float(self.steer_Kp2[ 0 ]), float(self.steer_Kp2[ 1 ]) ]
    self.steerKp1 = interp( cv_angle, cv, fKp1 )
    self.steerKp2 = interp( cv_angle, cv, fKp2 )
    self.steerKpV = [ float(self.steerKp1), float(self.steerKp2) ]

    # Ki
    fKi1 = [float(self.steer_Ki1[ 0 ]), float(self.steer_Ki1[ 1 ]) ]
    fKi2 = [float(self.steer_Ki2[ 0 ]), float(self.steer_Ki2[ 1 ]) ]
    self.steerKi1 = interp( cv_angle, cv, fKi1 )
    self.steerKi2 = interp( cv_angle, cv, fKi2 )
    self.steerKiV = [ float(self.steerKi1), float(self.steerKi2) ]

    # Kd
    fKd1 = [float(self.steer_Kd1[ 0 ]), float(self.steer_Kd1[ 1 ]) ]
    fKd2 = [float(self.steer_Kd2[ 0 ]), float(self.steer_Kd2[ 1 ]) ]
    self.steerKd1 = interp( cv_angle, cv, fKd1 )
    self.steerKd2 = interp( cv_angle, cv, fKd2 )    
    self.steerKdV = [ float(self.steerKd1), float(self.steerKd2) ]

    # kf
    fKf1 = [float(self.steer_Kf1[ 0 ]), float(self.steer_Kf1[ 1 ]) ]
    fKf2 = [float(self.steer_Kf2[ 0 ]), float(self.steer_Kf2[ 1 ]) ]
    self.steerKf1 = interp( cv_angle, cv, fKf1 )
    self.steerKf2 = interp( cv_angle, cv, fKf2 )


    xp = CP.lateralTuning.pid.kpBP
    fp = [float(self.steerKf1), float(self.steerKf2) ]
    self.steerKfV = interp( v_ego,  xp, fp )




    kBP = CP.lateralTuning.pid.kpBP
    self.pid.gain( (kBP, self.steerKpV), (kBP, self.steerKiV), k_f=self.steerKfV, k_d=(kBP, self.steerKdV) )



  def update(self, active, CS, CP, path_plan):
    self.angle_steers_des = path_plan.angleSteers    
    self.linear2_tune( CP, CS.vEgo )

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
