## @file romi_motor.py
# 
# This file contains the class to interface with the Romi motors.
# Objects of this class can be used to apply PWM to a given
# DC motor on one channel of the motor.

from pyb import Pin, Timer

## Interfaces with the Romi Motors
#
#  Rom class is used to create an object that can easily set motor speeds

class Rom: 
    
    ## Initialize a romi motor object so the motor can be easily controlled.
    # 
    #  This method initializes a motor object which can be used to set the duty cycle and
    #  enable the motor.
    #
    #  @param PWM_tim Timer for the PWM
    #  @param IN1 PWM pin
    #  @param DIR_Pin Output pin to control direction of motor
    #  @param EN_Pin Output pin to enable motor

    def __init__ (self, PWM_tim, IN1, DIR_Pin, EN_pin):
        self.IN1 = Pin(IN1, mode=Pin.OUT_PP)
        self.DIR_Pin = Pin(DIR_Pin, mode=Pin.OUT_PP)
        self.PWM_tim = PWM_tim
        self.EN_pin = Pin(EN_pin, mode=Pin.OUT_PP)
        self.pwm_ch_1 = PWM_tim.channel(1, pin=IN1, mode=Timer.PWM, pulse_width_percent = 0)
       
    ## Set the PWM duty cycle for the DC motor.
    #
    # This method sets the duty cycle to be sent
    # to the L6206 to a given level. Positive values
    # cause effort in one direction, negative values
    # in the opposite direction.
    #
    # @param duty A signed number holding the duty
    # cycle of the PWM signal sent to the motor
    def set_duty (self, duty):

        if duty > 0 :
            self.pwm_ch_1.pulse_width_percent(abs(duty))
            self.DIR_Pin.low()
        elif duty < 0:
            self.pwm_ch_1.pulse_width_percent(abs(duty))
            self.DIR_Pin.high()
        else:
            self.pwm_ch_1.pulse_width_percent(0)


    ## Enable one channel of the L6206.
    # This method sets the enable pin associated with one
    # channel of the L6206 high in order to enable that
    # channel of the motor driver.
    def enable (self):
        self.EN_pin.high()
        pass

