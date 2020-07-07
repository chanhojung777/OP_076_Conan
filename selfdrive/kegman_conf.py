import json
import os


json_file_name = '/data/atom_076.json'

class kegman_conf():
  def __init__(self, CP=None):
    self.config = None
    self.init = { "sR_KpV1":[0.11,0.13], "sR_KiV1":[0.008,0.015], "sR_KdV1":[0.0,0.0], "sR_KfV1":[0.000001,0.00003],  \
                  "sR_KpV2":[0.12,0.15], "sR_KiV2":[0.010,0.020], "sR_KdV2":[0.0,0.0], "sR_KfV2":[0.000001,0.00003], \
                  "sR_BPV":[0.0,0.0], "sR_boostV":[0.0,0.0], "cvBPV":[90,255], \
                  "cvSteerMaxV1":[255,200], "cvSteerDeltaUpV1":[3,2], "cvSteerDeltaDnV1":[5,3], \
                  "cvSteerMaxV2":[255,200], "cvSteerDeltaUpV2":[3,2], "cvSteerDeltaDnV2":[5,3], \
                  "steerRatio":11.5, "steerRateCost":0.4, \
                  "tire_stiffness_factor":1.0, \
                  "deadzone":0.0, "steerOffset":0.0, \
                  "steerLimitTimer":0.4,
                  "steerActuatorDelay":0.1 }



  def data_check(self, name, value ):
    if name not in self.config:
        self.config.update({name:value})
        self.element_updated = True


  def read_config(self):
    self.element_updated = False

    if os.path.isfile( json_file_name ):
      with open( json_file_name, 'r') as f:
        str_kegman = f.read()
        print( str_kegman )
        self.config = json.loads(str_kegman)

      for name in self.init:
        self.data_check( name, self.init[name] )

      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = self.init      
      self.write_config(self.config)

    return self.config

  def write_config(self, config):
    try:
      with open( json_file_name, 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod( json_file_name, 0o764)
    except IOError:
      os.mkdir('/data')
      with open( json_file_name, 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod( json_file_name, 0o764)
