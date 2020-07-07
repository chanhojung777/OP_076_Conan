
from  selfdrive.kegman_conf import kegman_conf

class AtomConf():
  def __init__(self, CP=None):
    self.kegman = kegman_conf()

    self.sR_BPV = [4.,30.]
    self.sR_BoostV = [0.0,1.5]

    self.sR_kpV1 = [0.11,0.12]
    self.sR_kiV1 = [0.008,0.01]
    self.sR_kdV1 = [0.0,0.0]
    self.sR_kfV1 = [0.000001,0.00001]

    self.sR_kpV2 = [0.13,0.15]
    self.sR_kiV2 = [0.015,0.02]
    self.sR_kdV2 = [0.0,0.0]
    self.sR_kfV2 = [0.00003,0.00003]

    self.cvBPV = [80,255]
    self.cvSteerMaxV1 = [255,200]
    self.cvSteerDeltaUpV1 = [3,2]
    self.cvSteerDeltaDnV1 = [5,3]

    self.cvSteerMaxV2 = [255,200]
    self.cvSteerDeltaUpV2 = [3,2]
    self.cvSteerDeltaDnV2 = [5,3]


    self.deadzone = 0.1
    self.steerOffset = 0.0
    self.steerRatio = 11.5
    self.steerRateCost = 0.4
    self.tire_stiffness_factor = 1.0
    self.steerActuatorDelay = 0.1
    self.steerLimitTimer = 0.4

    self.read_tune()


  def read_tune(self):
    conf = self.kegman.read_config()
    self.sR_BPV = conf['sR_BPV']
    self.sR_BoostV = conf['sR_boostV']

    self.sR_kpV1 = conf['sR_KpV1']
    self.sR_kiV1 = conf['sR_KiV1']
    self.sR_kdV1 = conf['sR_KdV1']
    self.sR_kfV1 = conf['sR_KfV1']

    self.sR_kpV2 = conf['sR_KpV2']
    self.sR_kiV2 = conf['sR_KiV2']
    self.sR_kdV2 = conf['sR_KdV2']
    self.sR_kfV2 = conf['sR_KfV2']


    self.cvBPV = conf['cvBPV']
    self.cvSteerMaxV1 = conf['cvSteerMaxV1']
    self.cvSteerDeltaUpV1 = conf['cvSteerDeltaUpV1']
    self.cvSteerDeltaDnV1 = conf['cvSteerDeltaDnV1']

    self.cvSteerMaxV2 = conf['cvSteerMaxV2']
    self.cvSteerDeltaUpV2 = conf['cvSteerDeltaUpV2']
    self.cvSteerDeltaDnV2 = conf['cvSteerDeltaDnV2']

    self.steerRatio = conf['steerRatio']
    self.steerRateCost = conf['steerRateCost']

    self.deadzone = conf['deadzone']
    self.steerOffset = conf['steerOffset']
    self.tire_stiffness_factor = conf['tire_stiffness_factor']
    self.steerActuatorDelay = conf['steerActuatorDelay']
    self.steerLimitTimer = conf['steerLimitTimer']
    