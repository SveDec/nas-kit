#!/usr/bin/ python3
# -*- coding:utf-8 -*-

import RPi.GPIO as GPIO
import signal
import subprocess
import time


# Settings

# Pulse Width Management
# Pin number (BCM)
FAN_PWM = 18
LED_PWM = 26
# PWM frequency
fan_pwn_freq = 100
led_pwn_freq = 1	# the LED will blink once per second

# Fan power percentage
FAN_MAX = 100
FAN_MIN = 0
FAN_ACTIVE_MIN = 20

# Fan activation threshold (temp)
FAN_THR = 70


# Classes

class GracefulKiller():
  global end_of_loop
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.sig_int)
    signal.signal(signal.SIGTERM, self.sig_term)

  def sig_int(self, signum, frame):
    print('Interruption')
    self.kill_now = True

  def sig_term(self, signum, frame):
    print('Termination')
    self.kill_now = True

class PID():
    def __init__(self, P=1, I=1, D=1, expect=0):
        self.P = float(P)
        self.I = float(I)
        self.D = float(D)
        self.expect = expect
        self.error = 0
        self.last_error = 0
        self.error_sum = 0

    @property
    def pval(self):
        return self.error

    @property
    def ival(self):
        self.error_sum += self.error
        return self.error_sum

    @property
    def dval(self):
        return self.error - self.last_error

    def run(self, value, mode="PID"):
        self.last_error = self.error
        self.error = value - self.expect
        # print(self.error, self.last_error, self.pval, self.P)
        result_p = self.P * self.pval
        result_i = self.I * self.ival
        result_d = self.D * self.dval
        mode = mode.upper()
        result = 0.0
        if "P" in mode:
            result += result_p
        if "I" in mode:
            result += result_i
        if "D" in mode:
            result += result_d
        return result


# Functions

def cpu_temperature():          # works with a normal user, returns temp in milliCelsius
    raw_cpu_temperature = subprocess.getoutput("cat /sys/class/thermal/thermal_zone0/temp") 
    cpu_temperature = round(float(raw_cpu_temperature)/1000,1)               # convert str to float
    #print(type(cpu_temperature), ' ',cpu_temperature)
    return cpu_temperature

def gpu_temperature():          # vcgencmd works only with root
    raw_gpu_temperature = subprocess.getoutput( '/opt/vc/bin/vcgencmd measure_temp' )
    gpu_temperature = round(float(raw_gpu_temperature.replace( 'temp=', '' ).replace( '\'C', '' )), 1)
    #print(type(gpu_temperature), ' ',gpu_temperature)
    return gpu_temperature



### Main thread

if __name__ =='__main__':

  ## Pin init

  GPIO.setwarnings(False) 
  GPIO.setmode(GPIO.BCM)
  output_list = [FAN_PWM,LED_PWM]
  GPIO.setup(output_list, GPIO.OUT)

  fan_pwm_pin = GPIO.PWM(FAN_PWM, fan_pwn_freq)
  led_pwm_pin = GPIO.PWM(LED_PWM, led_pwn_freq)

  fan_pwm_pin.start(0)
  led_pwm_pin.start(0)

  ## Loop

  end_of_loop = 0

  killer = GracefulKiller()

  # Calculates dc
  pid = PID(
    P = 0.5,
    I = 1,
    D = 1,
    expect = FAN_THR,
  )

  # Duty Cycle
  dc = 0

  while not killer.kill_now:
    temp = (cpu_temperature()+gpu_temperature())/2.0 
    dc += pid.run(temp, mode="PD")
    dc = min(FAN_MAX, max(FAN_MIN, dc))
    #print(temp,' - ',dc)
    if temp > FAN_THR:
      dc = min(FAN_ACTIVE_MIN, dc)
    fan_pwm_pin.ChangeDutyCycle(dc)
    led_pwm_pin.ChangeDutyCycle(dc)
    time.sleep(1)

  #print('Led and fan extinction')
  #time.sleep(1)
  fan_pwm_pin.ChangeDutyCycle(0)
  led_pwm_pin.ChangeDutyCycle(0)
  fan_pwm_pin.stop()
  led_pwm_pin.stop()
  # If we clean up, power sent to fans is set back to 100%
  #GPIO.cleanup()

