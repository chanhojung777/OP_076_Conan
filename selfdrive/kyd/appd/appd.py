#!/usr/bin/env python3

import time
import subprocess
import cereal
import threading
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

class AsyncTask:
  def __init__(self):
    self.softkey_is_running = False
    self.opkr_enable_softkey = False
    self.navigation_is_running = int( params.get("OpkrBootNavigation", encoding='utf8') )

  def all_kill(self, enable = 0 ):
    put_nonblocking('OpkrRunSoftkey', '0')
    for x in program:
      nPos = x[0]
      put_nonblocking( x[1], '0')
      if program_runing[nPos]:
        program_runing[nPos] = 0
        self.exec_app( enable, x[2], x[3])


  def exec_app(self, status, app, app_main):
    if status == 1:
      self.system("pm enable %s" % app)
      self.system("am start -n %s/%s" % (app, app_main))
      return True
    if status == 0:
      self.system("pm disable %s" % app)
      return False


  def system(self, cmd):
    try:
      # cloudlog.info("running %s" % cmd)
      subprocess.check_output(cmd, stderr=subprocess.STDOUT, shell=True)
    except subprocess.CalledProcessError as e:
      cloudlog.event("running failed",
                    cmd=e.cmd,
                    output=e.output[-1024:],
                    returncode=e.returncode)


  def Task(self, gctx=None):
    self.opkr_enable_softkey = int(params.get('OpkrRunSoftkey', encoding='utf8'))
    for x in program:
      nPos = x[0]        
      opkr_enable = int(params.get( x[1], encoding='utf8'))
      if self.navigation_is_running:
        if nPos == 2:
          opkr_enable = True

      if opkr_enable:
        print( '1:{} 2:{}'.format( x[1], x[2], program_runing[nPos] ) )
        self.all_kill()
        program_runing[nPos] = self.exec_app( opkr_enable, x[2], x[3])
        time.sleep(1)
        put_nonblocking( x[1], '0')
        if nPos == 2:
          self.opkr_enable_softkey = True
      else:
        time.sleep(0.1)


    self.navigation_is_running = 0
    # allow user to manually start/stop app
    if self.opkr_enable_softkey:
      put_nonblocking('OpkrRunSoftkey', '0')
      if not self.softkey_is_running:
        self.softkey_is_running = self.exec_app(self.opkr_enable_softkey, "com.gmd.hidesoftkeys", "com.gmd.hidesoftkeys.MainActivity")
      
    #threading.Timer( 0.3, self.Task ).start()
    time.sleep(0.3)




app = AsyncTask()
def main(gctx=None):
  app.all_kill()

  while True:
    app.Task()

if __name__ == "__main__":
  main()