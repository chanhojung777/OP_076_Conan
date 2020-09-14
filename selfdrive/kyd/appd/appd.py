#!/usr/bin/env python3

import time
import cereal.messaging as messaging
import subprocess
import cereal
ThermalStatus = cereal.log.ThermalData.ThermalStatus
from selfdrive.swaglog import cloudlog
from common.params import Params, put_nonblocking
params = Params()

program = (
  (0,"OpkrRunMixplorer", "com.mixplorer","com.mixplorer.activities.BrowseActivity"),
  (1,"OpkrRunQuickedit", "com.rhmsoft.edit.pro","com.rhmsoft.edit.activity.MainActivity"),
  (2,"OpkrRunNavigation", "com.skt.tmap.ku","com.skt.tmap.activity.TmapNaviActivity"),
  #("OpkrRunSoftkey", "com.gmd.hidesoftkeys","com.gmd.hidesoftkeys.MainActivity"),  
)


program_runing = [0,0,0,0]

def main(gctx=None):
  softkey_is_running = False

  opkr_enable_softkey = False

  navigation_is_running = True if params.get("OpkrBootNavigation", encoding='utf8') == "1" else False

  while True:
    for x in program:
      opkr_enable = int(params.get( x[1], encoding='utf8'))
      nPos = x[0]      
      if opkr_enable:
        print( '1:{} 2:{}'.format( x[1], x[2], program_runing[nPos] ) )        
        put_nonblocking( x[1], '0')
        if not program_runing[nPos]:
          program_runing[nPos] = exec_app( opkr_enable, x[2], x[3])
      elif program_runing[nPos]:
        program_runing[nPos] = 0
        exec_app( opkr_enable, x[2], x[3])



    # allow user to manually start/stop app
    opkr_enable_softkey = int(params.get('OpkrRunSoftkey', encoding='utf8'))
    if opkr_enable_softkey:
      put_nonblocking('OpkrRunSoftkey', '0')
      if not softkey_is_running:
        softkey_is_running = exec_app(opkr_enable_softkey, "com.gmd.hidesoftkeys", "com.gmd.hidesoftkeys.MainActivity")
       

    # every 3 seconds, we re-check status
    time.sleep(0.1)


def exec_app(status, app, app_main):
  if status == 1:
    system("pm enable %s" % app)
    system("am start -n %s/%s" % (app, app_main))
    return True
  if status == 0:
    system("pm disable %s" % app)
    return False


def system(cmd):
  try:
    # cloudlog.info("running %s" % cmd)
    subprocess.check_output(cmd, stderr=subprocess.STDOUT, shell=True)
  except subprocess.CalledProcessError as e:
    cloudlog.event("running failed",
                   cmd=e.cmd,
                   output=e.output[-1024:],
                   returncode=e.returncode)

if __name__ == "__main__":
  main()