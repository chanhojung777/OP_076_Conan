import numpy as np
from common.numpy_fast import clip, interp

import common.log as trace1


def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

class PIController():
  def __init__(self, k_p, k_i, k_f=1., pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, convert=None):
    self._k_p = k_p # proportional gain
    self._k_i = k_i # integral gain
    self._k_d = None    
    self.k_f = k_f  # feedforward gain

    self.time_cnt = 0
    self.errorPrev = 0	# History: Previous error

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.d_rate = 1.0 / rate
    self.sat_limit = sat_limit
    self.convert = convert

    self.reset()

    self.trPID = trace1.Loger("pid_ctrl")   

  def gain(self, k_p, k_i, k_f, k_d = None ):
    self._k_p = k_p # proportional gain
    self._k_i = k_i # integral gain
    self._k_d = k_d #  Derivative gain
    self.k_f = k_f  # feedforward gain    

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])    

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.f = 0.0
    self.d = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    self.p = error * self.k_p
    self.f = feedforward * self.k_f

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + self.f + i

      if self.convert is not None:
        control = self.convert(control, speed=self.speed)

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or \
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i


		# Compute the derivative output
    if self._k_d is not None:
      delta = (error - self.errorPrev) / self.d_rate
      self.d = delta * self.k_d

    # input 
    #if self._k_d is not None:
    #  dInput = setpoint - self.prevInput
    #  self.d = -self.k_d * (dInput / self.d_rate)
    #  self.prevInput = setpoint

    control = self.p + self.f + self.i + self.d
    if self.convert is not None:
      control = self.convert(control, speed=self.speed)

    self.saturated = self._check_saturation(control, check_saturation, error)

    self.control = clip(control, self.neg_limit, self.pos_limit)

    self.errorPrev = error

    self.time_cnt += 1
    if self.time_cnt > 10:
      self.time_cnt = 0

    str1 = 'speed={:.2f} control={:.5f} a={:.2f}/{:.2f}/{:.0f} p={:.5f} f={:.5f} i={:.5f} d={:.5f}'.format( speed*3.6, self.control, setpoint, measurement, override, self.p, self.f, self.i, self.d )
    self.trPID.add( str1 )
      
    return self.control
