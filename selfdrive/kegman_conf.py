import json
import os

class kegman_conf():
  def __init__(self, CP=None):
    self.conf = self.read_config()
    if CP is not None:
      self.init_config(CP)

  def init_config(self, CP):
    write_conf = False
    if self.conf['tuneGernby'] != "1":
      self.conf['tuneGernby'] = str(1)
      write_conf = True
	
    # only fetch Kp, Ki, Kf sR and sRC from interface.py if it's a PID controlled car
    if CP.lateralTuning.which() == 'pid':
      if self.conf['Kp'] == "-1":
        self.conf['Kp'] = str(round(CP.lateralTuning.pid.kpV[0],3))
        write_conf = True
      if self.conf['Ki'] == "-1":
        self.conf['Ki'] = str(round(CP.lateralTuning.pid.kiV[0],3))
        write_conf = True
      if self.conf['Kf'] == "-1":
        self.conf['Kf'] = str('{:f}'.format(CP.lateralTuning.pid.kf))
        write_conf = True
    
    if self.conf['steerRatio'] == "-1":
      self.conf['steerRatio'] = str(round(CP.steerRatio,3))
      write_conf = True
    
    if self.conf['steerRateCost'] == "-1":
      self.conf['steerRateCost'] = str(round(CP.steerRateCost,3))
      write_conf = True

    if write_conf:
      self.write_config(self.config)

  def read_config(self):
    self.element_updated = False

    if os.path.isfile('/data/kegman.json'):
      with open('/data/kegman.json', 'r') as f:
        str_kegman = f.read()
        print( str_kegman )
        self.config = json.loads(str_kegman)

      if "battPercOff" not in self.config:
        self.config.update({"battPercOff":"30"})
        self.config.update({"carVoltageMinEonShutdown":"11800"})
        self.element_updated = True

      if "tuneGernby" not in self.config:
        self.config.update({"tuneGernby":"1"})
        self.config.update({"deadzone":"0.0"})
        self.config.update({"steerAngleOffset":"0.0"})


      if "Kp" not in self.config:
        self.config.update({"Kp":"-1"})
        self.config.update({"Ki":"-1"})
        self.config.update({"Kf":"-1"})
        self.element_updated = True

	
      if "steerRatio" not in self.config:
        self.config.update({"steerRatio":"-1"})
        self.config.update({"steerRateCost":"-1"})
        self.element_updated = True
		
      if "sR_boost" not in self.config:
        self.config.update({"sR_boost":"0"})
        self.config.update({"sR_BP0":"0"})
        self.config.update({"sR_BP1":"0"})
        self.config.update({"sR_time":"1"})
        self.element_updated = True

      if "sR_Kp" not in self.config:
        self.config.update({"sR_Kp":"0.25"})
        self.config.update({"sR_Ki":"0.05"})
        self.config.update({"sR_Kf":"0.00005"})
        self.element_updated = True



      if self.element_updated:
        print("updated")
        self.write_config(self.config)

    else:
      self.config = {"battChargeMin":"70", "battChargeMax":"80", \
                    "battPercOff":"30", "carVoltageMinEonShutdown":"11800", \
                    "tuneGernby":"1", "deadzone":"0.0",\
                    "Kp":"-1", "Ki":"-1", "Kf":"-1",  \
                    "steerRatio":"-1", "steerRateCost":"-1", \
                    "sR_boost":"0", "sR_BP0":"0", "sR_BP1":"0", "sR_time":"1", \
                    "sR_Kp":"0.25", "sR_Ki":"0.05", "sR_Kf":"0.00005"}


      self.write_config(self.config)
    return self.config

  def write_config(self, config):
    try:
      with open('/data/kegman.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kegman.json", 0o764)
    except IOError:
      os.mkdir('/data')
      with open('/data/kegman.json', 'w') as f:
        json.dump(self.config, f, indent=2, sort_keys=True)
        os.chmod("/data/kegman.json", 0o764)
