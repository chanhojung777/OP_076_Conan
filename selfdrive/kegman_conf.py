import json
import os


json_file_name = '/data/kegman_076.json'

class kegman_conf():
  def __init__(self, CP=None):
    #self.conf = self.read_config()
    self.config = None

  def read_config(self):
    self.element_updated = False

    if os.path.isfile( json_file_name ):
      with open( json_file_name, 'r') as f:
        str_kegman = f.read()
        print( str_kegman )
        self.config = json.loads(str_kegman)


      if "steerOffset" not in self.config:
        self.config.update({"deadzone":"0.1"})
        self.config.update({"steerOffset":"0.0"})

      if "Kp" not in self.config:
        self.config.update({"Kp":"0.11"})
        self.config.update({"Ki":"0.1"})
        self.config.update({"Kd":"0.0"})
        self.config.update({"Kf":"0.000001"})
        self.element_updated = True

      if "Kp2" not in self.config:
        self.config.update({"Kp2":"0.11"})
        self.config.update({"Ki2":"0.1"})
        self.config.update({"Kd2":"0.0"})
        self.config.update({"Kf2":"0.000001"})
        self.element_updated = True

      if "sR_Kp" not in self.config:
        self.config.update({"sR_Kp":"0.25"})
        self.config.update({"sR_Ki":"0.05"})
        self.config.update({"sR_Kd":"0.0"})
        self.config.update({"sR_Kf":"0.00005"})
        self.element_updated = True

      if "sR_Kp2" not in self.config:
        self.config.update({"sR_Kp2":"0.25"})
        self.config.update({"sR_Ki2":"0.05"})
        self.config.update({"sR_Kd2":"0.0"})
        self.config.update({"sR_Kf2":"0.00005"})
        self.element_updated = True        
	
      if "steerRatio" not in self.config:
        self.config.update({"steerRatio":"12.5"})
        self.config.update({"steerRateCost":"0.5"})
        self.config.update({"tire_stiffness_factor":"1.0"})
        self.element_updated = True
		
      if "sR_boost" not in self.config:
        self.config.update({"sR_boost":"0"})
        self.config.update({"sR_BP0":"0"})
        self.config.update({"sR_BP1":"0"})
        self.element_updated = True


      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = {"Kp":"0.11", "Ki":"0.008", "Kd":"0.0", "Kf":"0.000001",  \
                    "Kp2":"0.12", "Ki2":"0.01", "Kd2":"0.0", "Kf2":"0.000001",  \
                    "sR_Kp":"0.13", "sR_Ki":"0.015", "sR_Kd":"0.0", "sR_Kf":"0.00003", \
                    "sR_Kp2":"0.15", "sR_Ki2":"0.02", "sR_Kd2":"0.0", "sR_Kf2":"0.00003", \
                    "sR_BP0":"3", "sR_BP1":"15", \
                    "sR_boost":"1.5", "deadzone":"0.0", "steerOffset":"0.0", \
                    "steerRatio":"10.5", "steerRateCost":"0.4", "tire_stiffness_factor":"1.0",\
                    "LearnerParams":"0" }
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
