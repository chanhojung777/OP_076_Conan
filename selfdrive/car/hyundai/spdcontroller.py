import math
import numpy as np

from cereal import log
import cereal.messaging as messaging


from cereal import log
import cereal.messaging as messaging
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.planner import calc_cruise_accel_limits
from selfdrive.controls.lib.speed_smoother import speed_smoother
from selfdrive.controls.lib.long_mpc import LongitudinalMpc

from selfdrive.car.hyundai.values import Buttons, SteerLimitParams
from common.numpy_fast import clip, interp

from selfdrive.config import RADAR_TO_CAMERA


import common.log as trace1
import common.CTime1000 as tm
import common.MoveAvg as moveavg1



cv_Raio = 0.8
cv_Dist = -5

MAX_SPEED = 255.0

LON_MPC_STEP = 0.2  # first step is 0.2s
MAX_SPEED_ERROR = 2.0
AWARENESS_DECEL = -0.2     # car smoothly decel at .2m/s^2 when user is distracted

# lookup tables VS speed to determine min and max accels in cruise
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MIN_V = [-1.0, -.8, -.67, -.5, -.30]
_A_CRUISE_MIN_BP = [0., 5.,  10., 20.,  40.]

# need fast accel at very low speed for stop and go
# make sure these accelerations are smaller than mpc limits
_A_CRUISE_MAX_V = [1.2, 1.2, 0.65, .4]
_A_CRUISE_MAX_V_FOLLOWING = [1.6, 1.6, 0.65, .4]
_A_CRUISE_MAX_BP = [0.,  6.4, 22.5, 40.]

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

# 75th percentile
SPEED_PERCENTILE_IDX = 7

def limit_accel_in_turns(v_ego, angle_steers, a_target, steerRatio, wheelbase):
    """
    This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
    this should avoid accelerating when losing the target in turns
    """

    a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego**2 * angle_steers * CV.DEG_TO_RAD / (steerRatio * wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max**2 - a_y**2, 0.))

    return [a_target[0], min(a_target[1], a_x_allowed)]


class SpdController():
    def __init__(self):
        self.long_control_state = 0  # initialized to off

        self.seq_step_debug = 0
        self.long_curv_timer = 0

        self.path_x = np.arange(192)

        self.traceSC = trace1.Loger("SPD_CTRL")

        self.wheelbase = 2.845
        self.steerRatio = 12.5  # 12.5

        self.v_model = 0
        self.a_model = 0
        self.v_cruise = 0
        self.a_cruise = 0

        self.l_poly = []
        self.r_poly = []

        self.movAvg = moveavg1.MoveAvg()
        self.Timer1 = tm.CTime1000("SPD")
        self.time_no_lean = 0

        self.SC = trace1.Loger("spd")

        self.wait_timer2 = 0



        self.cruise_set_speed_kph = 0
        self.curise_set_first = 0
        self.curise_sw_check = 0
        self.prev_clu_CruiseSwState = 0    

        self.prev_VSetDis  = 0

        self.cruise_set_mode = 0

    def reset(self):
        self.v_model = 0
        self.a_model = 0
        self.v_cruise = 0
        self.a_cruise = 0


    def calc_va(self, sm, v_ego):
        md = sm['model']
        if len(md.path.poly):
            path = list(md.path.poly)

            self.l_poly = np.array(md.leftLane.poly)
            self.r_poly = np.array(md.rightLane.poly)
            #self.p_poly = np.array(md.path.poly)

            # Curvature of polynomial https://en.wikipedia.org/wiki/Curvature#Curvature_of_the_graph_of_a_function
            # y = a x^3 + b x^2 + c x + d, y' = 3 a x^2 + 2 b x + c, y'' = 6 a x + 2 b
            # k = y'' / (1 + y'^2)^1.5
            # TODO: compute max speed without using a list of points and without numpy
            y_p = 3 * path[0] * self.path_x**2 + \
                2 * path[1] * self.path_x + path[2]
            y_pp = 6 * path[0] * self.path_x + 2 * path[1]
            curv = y_pp / (1. + y_p**2)**1.5

            a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
            v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
            model_speed = np.min(v_curvature)
            # Don't slow down below 20mph
            model_speed = max(30.0 * CV.MPH_TO_MS, model_speed)

            model_sum = curv[2] * 1000.  #np.sum( curv, 0 )

            model_speed = model_speed * CV.MS_TO_KPH
            if model_speed > MAX_SPEED:
                model_speed = MAX_SPEED
        else:
            model_speed = MAX_SPEED
            model_sum = 0

        model_speed = self.movAvg.get_min(model_speed, 10)

        return model_speed, model_sum


    def update_cruiseSW(self, CS ):
        set_speed_kph = self.cruise_set_speed_kph
        delta_vsetdis = 0
        if CS.pcm_acc_status:
            delta_vsetdis = abs(CS.VSetDis - self.prev_VSetDis)
            if self.prev_clu_CruiseSwState != CS.cruise_buttons:
                if CS.cruise_buttons:
                    self.prev_VSetDis = int(CS.VSetDis)
                elif CS.driverOverride:
                    set_speed_kph = int(CS.VSetDis)          
                elif self.prev_clu_CruiseSwState == Buttons.RES_ACCEL:   # up 
                    if self.curise_set_first:
                        self.curise_set_first = 0
                        set_speed_kph =  int(CS.VSetDis)
                    elif delta_vsetdis > 5:
                        set_speed_kph = CS.VSetDis
                    elif not self.curise_sw_check:
                        set_speed_kph += 1
                elif self.prev_clu_CruiseSwState == Buttons.SET_DECEL:  # dn
                    if self.curise_set_first:
                        self.curise_set_first = 0
                        set_speed_kph = int(CS.clu_Vanz)
                    elif delta_vsetdis > 5:
                        set_speed_kph = int(CS.VSetDis)
                    elif not self.curise_sw_check:
                        set_speed_kph -= 1

                self.prev_clu_CruiseSwState = CS.cruise_buttons
            elif CS.cruise_buttons and delta_vsetdis > 0:
                self.curise_sw_check = True
                set_speed_kph = int(CS.VSetDis)
        else:
            self.curise_sw_check = False
            self.curise_set_first = 1
            self.prev_VSetDis = int(CS.VSetDis)
            set_speed_kph = CS.VSetDis
            if self.prev_clu_CruiseSwState != CS.cruise_buttons:  # MODE 전환.
                if CS.cruise_buttons == Buttons.CANCEL: 
                    self.cruise_set_mode += 1
                if self.cruise_set_mode > 3:
                    self.cruise_set_mode = 0
                self.prev_clu_CruiseSwState = CS.cruise_buttons
            
        trace1.cruise_set_mode = self.cruise_set_mode

        if set_speed_kph < 30:
            set_speed_kph = 30

        self.cruise_set_speed_kph = set_speed_kph
        return set_speed_kph

    def speed_control(self, CS, v_ego_kph, sm, actuators, dRel, yRel, vRel ):
        if CS.driverOverride == 2 or not CS.pcm_acc_status or CS.cruise_buttons == Buttons.RES_ACCEL or CS.cruise_buttons == Buttons.SET_DECEL:
            self.resume_cnt = 0
            self.btn_type = Buttons.NONE
            self.wait_timer2 = 10
            self.active_timer2 = 0
        elif self.wait_timer2:
            self.wait_timer2 -= 1
        else:
            btn_type, clu_speed = self.update( v_ego_kph, CS, sm, actuators, dRel, yRel, vRel )   # speed controller spdcontroller.py

            if CS.clu_Vanz < 5:
                self.btn_type = Buttons.NONE
            elif self.btn_type != Buttons.NONE:
                pass
            elif btn_type != Buttons.NONE:
                self.resume_cnt = 0
                self.active_timer2 = 0
                self.btn_type = btn_type
                self.clu_speed = clu_speed

            if self.btn_type != Buttons.NONE:
                self.active_timer2 += 1
                if self.active_timer2 > 10:
                    self.wait_timer2 = 5
                    self.resume_cnt = 0
                    self.active_timer2 = 0
                    self.btn_type = Buttons.NONE          
                else:
                    return 1
        return  0   

    @staticmethod
    def get_lead( sm ):
        lead_msg = sm['model'].lead
        if lead_msg.prob > 0.5:
            dRel = float(lead_msg.dist - RADAR_TO_CAMERA)
            yRel = float(lead_msg.relY)
            vRel = float(lead_msg.relVel)
        else:
            dRel = 150
            yRel = 0
            vRel = 0


        return dRel, yRel, vRel



    def get_tm_speed(self, CS, set_time, add_val, safety_dis=5):
        time = int(set_time)

        delta_speed = CS.VSetDis - CS.clu_Vanz
        set_speed = int(CS.VSetDis) + add_val
        
        if add_val > 0:  # 증가
            if delta_speed > safety_dis:
                time = 100
        else:
            if delta_speed < -safety_dis:
                time = 100

        return time, set_speed

    def update_lead(self, CS,  dRel, yRel, vRel):
        lead_set_speed = self.cruise_set_speed_kph
        lead_wait_cmd = 600
        self.seq_step_debug = 0
        if int(self.cruise_set_mode) != 2:
            return lead_wait_cmd, lead_set_speed

        self.seq_step_debug = 1
        #dRel, yRel, vRel = self.get_lead( sm, CS )
        if CS.lead_distance < 150:
            dRel = CS.lead_distance
            vRel = CS.lead_objspd

        dst_lead_distance = (CS.clu_Vanz*cv_Raio)   # 유지 거리.

        if dst_lead_distance > 100:
            dst_lead_distance = 100
        elif dst_lead_distance < 50:
            dst_lead_distance = 50

        if dRel < 150:
            self.time_no_lean = 0
            d_delta = dRel - dst_lead_distance
            lead_objspd = vRel  # 선행차량 상대속도.
        else:
            d_delta = 0
            lead_objspd = 0

        # 가속이후 속도 설정.
        if CS.driverAcc_time:
          lead_set_speed = CS.clu_Vanz
          lead_wait_cmd = 100
          self.seq_step_debug = 2
        elif CS.VSetDis > 70 and lead_objspd < -20:
            self.seq_step_debug = 3
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -2)
        elif CS.VSetDis > 60 and lead_objspd < -15:
            self.seq_step_debug = 4
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -2)      
        # 1. 거리 유지.
        elif d_delta < 0:
            # 선행 차량이 가까이 있으면.
            dVanz = dRel - CS.clu_Vanz
            self.seq_step_debug = 5
            if lead_objspd >= 0:    # 속도 유지 시점 결정.
                self.seq_step_debug = 6
                if CS.VSetDis > (CS.clu_Vanz + 10):
                    lead_wait_cmd = 200
                    lead_set_speed = CS.VSetDis - 1  # CS.clu_Vanz + 5
                    if lead_set_speed < 40:
                        lead_set_speed = 40
                else:
                    lead_set_speed = int(CS.VSetDis)

            elif lead_objspd < -30 or (dRel < 50 and CS.VSetDis > 60 and lead_objspd < -5):
                self.seq_step_debug = 7
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -2)
            elif lead_objspd < -20 or (dRel < 70 and CS.VSetDis > 60 and lead_objspd < -5):
                self.seq_step_debug = 8
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 20, -2)
            elif lead_objspd < -10:
                self.seq_step_debug = 9
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, -1)
            elif lead_objspd < 0:
                self.seq_step_debug = 10
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 80, -1)
            else:
                self.seq_step_debug = 11
                lead_set_speed = int(CS.VSetDis)

        # 선행차량이 멀리 있으면.
        elif lead_objspd < -20:
            self.seq_step_debug = 12
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 15, -2)
        elif lead_objspd < -10:
            self.seq_step_debug = 13
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, -1)
        elif lead_objspd < -5:
            self.seq_step_debug = 14
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 150, -1)
        elif lead_objspd < -1:
            self.seq_step_debug = 15
            lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 200, -1)
        elif self.cruise_set_speed_kph > CS.clu_Vanz:
            self.seq_step_debug = 16
            # 선행 차량이 가속하고 있으면.
            if dRel >= 150:
                self.seq_step_debug = 17
                lead_wait_cmd, lead_set_speed = self.get_tm_speed( CS, 200, 1 )
            elif lead_objspd < cv_Dist:
                self.seq_step_debug = 18
                lead_set_speed = int(CS.VSetDis)
            elif lead_objspd < 2:
                self.seq_step_debug = 19
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 500, 1)
            elif lead_objspd < 5:
                self.seq_step_debug = 20
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 300, 1)
            elif lead_objspd < 10:
                self.seq_step_debug = 21
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 200, 1)
            elif lead_objspd < 30:
                self.seq_step_debug = 22
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 100, 1)                
            else:
                self.seq_step_debug = 23
                lead_wait_cmd, lead_set_speed = self.get_tm_speed(CS, 50, 1)

        return lead_wait_cmd, lead_set_speed

    def update_curv(self, CS, sm, model_speed):
        wait_time_cmd = 0
        set_speed = self.cruise_set_speed_kph

        
        # 2. 커브 감속.
        if self.cruise_set_speed_kph >= 70:
            if model_speed < 80:
                set_speed = self.cruise_set_speed_kph - 15
                wait_time_cmd = 100
            elif model_speed < 110:  # 6도
                set_speed = self.cruise_set_speed_kph - 10
                wait_time_cmd = 150
            elif model_speed < 160:  # 3 도
                set_speed = self.cruise_set_speed_kph - 5
                wait_time_cmd = 200

            if set_speed > model_speed:
                set_speed = model_speed

        return wait_time_cmd, set_speed

    def update(self, v_ego_kph, CS, sm, actuators, dRel, yRel, vRel):
        btn_type = Buttons.NONE
        #lead_1 = sm['radarState'].leadOne
        long_wait_cmd = 500
        set_speed = self.cruise_set_speed_kph

        dec_step_cmd = 0

        if self.long_curv_timer < 600:
            self.long_curv_timer += 1


        # 선행 차량 거리유지
        lead_wait_cmd, lead_set_speed = self.update_lead( CS,  dRel, yRel, vRel)  

        # 커브 감속.
        model_speed = self.calc_va( CS.out.vEgo )
        curv_wait_cmd, curv_set_speed = self.update_curv(CS, sm, model_speed)

        if curv_wait_cmd != 0:
            if lead_set_speed > curv_set_speed:
                dec_step_cmd = 1
                set_speed = curv_set_speed
                long_wait_cmd = curv_wait_cmd
            else:
                set_speed = lead_set_speed
                long_wait_cmd = lead_wait_cmd
        else:
            set_speed = lead_set_speed
            long_wait_cmd = lead_wait_cmd

        if set_speed > self.cruise_set_speed_kph:
            set_speed = self.cruise_set_speed_kph
        elif set_speed < 30:
            set_speed = 30

        # control process
        target_set_speed = set_speed
        delta = int(set_speed) - int(CS.VSetDis)
        if dec_step_cmd == 0 and delta < -1:
            if delta < -3:
                dec_step_cmd = 4
            elif delta < -2:
                dec_step_cmd = 3
            else:
                dec_step_cmd = 2
        else:
            dec_step_cmd = 1


        if self.long_curv_timer < long_wait_cmd:
            pass
        elif CS.driverOverride == 1:  # 가속패달에 의한 속도 설정.
            if self.cruise_set_speed_kph > CS.clu_Vanz:
                delta = int(CS.clu_Vanz) - int(CS.VSetDis)
                if delta > 1:
                    set_speed = CS.clu_Vanz
                    btn_type = Buttons.SET_DECEL
        elif delta <= -1:
            set_speed = CS.VSetDis - dec_step_cmd
            btn_type = Buttons.SET_DECEL
            self.long_curv_timer = 0
        elif delta >= 1 and (model_speed > 200 or CS.clu_Vanz < 70):
            set_speed = CS.VSetDis + dec_step_cmd
            btn_type = Buttons.RES_ACCEL
            self.long_curv_timer = 0            
            if set_speed > self.cruise_set_speed_kph:
                set_speed = self.cruise_set_speed_kph
        if self.cruise_set_mode == 0:
            btn_type = Buttons.NONE



        str3 = 'SET={:3.0f} DST={:3.0f}  SD={:.0f} DA={:.0f}/{:.0f}/{:.0f} DG={}/{:.0f}'.format(
            set_speed, target_set_speed, CS.VSetDis, CS.driverAcc_time, long_wait_cmd, self.long_curv_timer, self.seq_step_debug, dec_step_cmd )

        str4 = ' CS={:.1f}/{:.1f} '.format(  CS.lead_distance, CS.lead_objspd )

        str5 = str3 +  str4
        trace1.printf2( str5 )

        return btn_type, set_speed
