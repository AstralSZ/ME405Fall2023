# -*- coding: utf-8 -*-
"""
Created on Mon Oct 16 13:48:08 2023

@author: Say Zhee
"""
from pyb import Pin, Timer


class Rom:
    '''!@brief A driver class for one channel of the L6206.
    @details Objects of this class can be used to apply PWM to a given
    DC motor on one channel of the L6206 from ST Microelectronics.
    '''

    def __init__ (self, PWM_tim, IN1, DIR_Pin, EN_pin):
        self.IN1 = Pin(IN1, mode=Pin.OUT_PP)
        self.DIR_Pin = Pin(DIR_Pin, mode=Pin.OUT_PP)
        self.PWM_tim = PWM_tim
        self.EN_pin = Pin(EN_pin, mode=Pin.OUT_PP)
        self.pwm_ch_1 = PWM_tim.channel(1, pin=IN1, mode=Timer.PWM, pulse_width_percent = 0)
       
        '''!@brief Initializes and returns an object associated with a DC motor.
        '''

    def set_duty (self, duty):
        '''!@brief Set the PWM duty cycle for the DC motor.
        @details This method sets the duty cycle to be sent
        to the L6206 to a given level. Positive values
        cause effort in one direction, negative values
        in the opposite direction.
        @param duty A signed number holding the duty
        cycle of the PWM signal sent to the L6206
        '''
        if duty > 0 :
            self.pwm_ch_1.pulse_width_percent(abs(duty))
            self.DIR_Pin.low()
        elif duty < 0:
            self.pwm_ch_1.pulse_width_percent(abs(duty))
            self.DIR_Pin.high()
        else:
            self.pwm_ch_1.pulse_width_percent(0)

    def enable (self):
        self.EN_pin.high()
        '''!@brief Enable one channel of the L6206.
        @details This method sets the enable pin associated with one
        channel of the L6206 high in order to enable that
        channel of the motor driver.
        '''
        pass

