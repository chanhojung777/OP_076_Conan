
from  kegman_conf import kegman_conf

class AtomConf():
  def __init__(self, CP=None):
    self.kegman = kegman_conf()

    self.sr_boost_bp = [4.,30.]
    self.sr_boost_range = [0.,1.5]

    self.steer_Kp1 = [0.11,0.12]
    self.steer_Ki1 = [0.008,0.01]
    self.steer_Kf1 = [0.000001,0.00001]

    self.steer_Kp2 = [0.13,0.15]
    self.steer_Ki2 = [0.015,0.02]
    self.steer_Kf2 = [0.00003,0.00003]

    self.deadzone = 0.1
    self.steerOffset = 0.0
    self.steerRatio = 10.5
    self.steerRateCost = 4.0


  def read_tune(self):
    conf = self.kegman.read_config()
    self.sr_boost_bp = [ float(conf['sR_BP0']), float(conf['sR_BP1']) ]
    self.sr_boost_range = [ 0.0, float(conf['sR_boost']) ]
    self.steer_Kp1 = [ float(conf['Kp']), float(conf['sR_Kp']) ]
    self.steer_Ki1 = [ float(conf['Ki']), float(conf['sR_Ki']) ]
    self.steer_Kf1 = [ float(conf['Kf']), float(conf['sR_Kf']) ]

    self.steer_Kp2 = [ float(conf['Kp2']), float(conf['sR_Kp2']) ]
    self.steer_Ki2 = [ float(conf['Ki2']), float(conf['sR_Ki2']) ]
    self.steer_Kf2 = [ float(conf['Kf2']), float(conf['sR_Kf2']) ]

    self.deadzone = float(conf['deadzone'])
    self.steerOffset = float(conf['steerOffset'])
    self.steerRatio = float(conf['steerRatio'])
    self.steerRateCost = float(conf['steerRateCost'])

