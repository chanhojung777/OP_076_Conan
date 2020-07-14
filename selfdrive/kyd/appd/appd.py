#!/usr/bin/env python3

import time
import cereal.messaging as messaging
import subprocess
import cereal
ThermalStatus = cereal.log.ThermalData.ThermalStatus
from selfdrive.swaglog import cloudlog
from common.params import Params, put_nonblocking
params = Params()

mixplorer = "com.mixplorer"
mixplorer_main = "com.mixplorer.activities.BrowseActivity"

quickedit = "com.rhmsoft.edit.pro"
quickedit_main = "com.rhmsoft.edit.activity.MainActivity"


tmap = "com.skt.tmap.ku"
tmap_main = "com.skt.tmap.activity.TmapNaviActivity"


softkey = "com.gmd.hidesoftkeys"
softkey_main = "com.gmd.hidesoftkeys.MainActivity"

offroad = "ai.comma.plus.offroad"
offroad_main = ".MainActivity"

def main(gctx=None):

  opkr_enable_mixplorer = True #if params.get('OpkrEnableMixplorer', encoding='utf8') == "1" else False
  opkr_enable_quickedit = True #if params.get("OpkrEnableQuickedit", encoding='utf8') == "1" else False
  opkr_enable_tmap = True #if params.get("OpkrEnableTmap", encoding='utf8') == "1" else False
  opkr_boot_tmap = True if params.get("OpkrBootTmap", encoding='utf8') == "1" else False
  opkr_enable_softkey = True #if params.get("OpkrEnableSoftkey", encoding='utf8') == "1" else False

  
  mixplorer_is_running = False
  quickedit_is_running = False
  tmap_is_running = False
  softkey_is_running = False

  allow_auto_boot = True
  last_started = False
  frame = 0
  start_delay = None
  stop_delay = None

  put_nonblocking('OpkrRunMixplorer', '0')
  put_nonblocking('OpkrRunQuickedit', '0')
  put_nonblocking('OpkrRunTmap', '0')
  put_nonblocking('OpkrRunSoftkey', '0')

  # we want to disable all app when boot
  system("pm disable %s" % mixplorer)
  system("pm disable %s" % quickedit)
  system("pm disable %s" % tmap)
  system("pm disable %s" % softkey)

  thermal_sock = messaging.sub_sock('thermal')

  while opkr_enable_mixplorer or opkr_enable_quickedit  or opkr_enable_tmap  or opkr_enable_softkey:

    # allow user to manually start/stop app
    if opkr_enable_mixplorer:
      status = params.get('OpkrRunMixplorer', encoding='utf8')
      if not status == "0":
        if not softkey_is_running:
          softkey_is_running = exec_app(status, softkey, softkey_main)
          put_nonblocking('OpkrRunSoftkey', '0')
        mixplorer_is_running = exec_app(status, mixplorer, mixplorer_main)
        put_nonblocking('OpkrRunMixplorer', '0')

    if opkr_enable_quickedit:
      status = params.get('OpkrRunQuickedit', encoding='utf8')
      if not status == "0":
        if not softkey_is_running:
          softkey_is_running = exec_app(status, softkey, softkey_main)
          put_nonblocking('OpkrRunSoftkey', '0')
        quickedit_is_running = exec_app(status, quickedit, quickedit_main)
        put_nonblocking('OpkrRunQuickedit', '0')

    if opkr_enable_softkey:
      status = params.get('OpkrRunSoftkey', encoding='utf8')
      if not status == "0":
        softkey_is_running = exec_app(status, softkey, softkey_main)
        put_nonblocking('OpkrRunSoftkey', '0')




    if opkr_enable_tmap:
      status = params.get('OpkrRunTmap', encoding='utf8')
      if not status == "0":
        if not softkey_is_running:
          softkey_is_running = exec_app(status, softkey, softkey_main)
          put_nonblocking('OpkrRunSoftkey', '0')
        tmap_is_running = exec_app(status, tmap, tmap_main)
        put_nonblocking('OpkrRunTmap', '0')

 

    msg = messaging.recv_sock(thermal_sock, wait=True)
    started = msg.thermal.started
    # car on
    if started:
      stop_delay = None
      if start_delay is None:
        start_delay = frame + 5


      if opkr_boot_tmap and frame > start_delay:
        if not tmap_is_running:
          if not softkey_is_running:
            softkey_is_running = exec_app(status, softkey, softkey_main)
            put_nonblocking('OpkrRunSoftkey', '0')
          tmap_is_running = exec_app('1', tmap, tmap_main)
          put_nonblocking('OpkrRunTmap', '0')


        # Logic:
        # if temp reach red, we disable all 3rd party apps.
        # once the temp drop below yellow, we then re-enable them
        #
        # set allow_auto_boot back to True once the thermal status is < yellow
        # kill mixplorer when car started
      #if mixplorer_is_running:
      #  mixplorer_is_running = exec_app('0', mixplorer, mixplorer_main)
      #if quickedit_is_running:
      #  quickedit_is_running = exec_app('0', quickedit, quickedit_main)

    # car off
    else:
      start_delay = None
      if stop_delay is None:
        stop_delay = frame + 30

    # if car state changed, we remove manual control state

    last_started = started
    frame += 3
    # every 3 seconds, we re-check status
    time.sleep(3)

def exec_app(status, app, app_main):
  if status == "1":
    system("pm enable %s" % app)
    system("am start -n %s/%s" % (app, app_main))
    return True
  if status == "0":
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