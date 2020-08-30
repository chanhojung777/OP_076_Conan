import json
import os


json_file_name = '/data/atom_0761.json'

class kegman_conf():
  def __init__(self, CP=None):
    self.config = None
    self.init = { 
        "tun_type": "lqr",
        "sR_KPH": [30,60],
        "sR_BPV": [[-5,0,5],[-10,0,10]],
        "sR_steerRatioV": [[13.95,13.85,13.95],[13.95,13.85,13.95]],
        "sR_ActuatorDelayV": [[0.25,0.5,0.25],[0.25,0.8,0.25]],
        "sR_pid_KiV": [[0.02,0.01,0.02],[0.03,0.02,0.03]],
        "sR_pid_KpV": [[0.20,0.15,0.20],[0.25,0.20,0.25]],
        "sR_pid_deadzone": 0.1,
        "sR_lqr_kiV": [[0.01,0.01,0.01],[0.02,0.02,0.02]],
        "sR_lqr_scaleV": [[2000,2000,2000],[1900,2000,1900]],
        "cv_KPH": [0],
        "cv_BPV": [[150,255]],
        "cv_sMaxV": [[255,200]],
        "cv_sdUPV": [[3,3]],
        "cv_sdDNV": [[7,5]],
        "steerOffset": 0.0,
        "steerRateCost": 0.5,
        "steerLimitTimer": 0.8,
        "cameraOffset":0.0
         }


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
