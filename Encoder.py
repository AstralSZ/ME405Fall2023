# -*- coding: utf-8 -*-
"""
Created on Mon Oct 16 13:47:39 2023

@author: Say Zhee
"""
from pyb import Pin, Timer
import time

class Encoder:
    '''!@brief Interface with quadrature encoders
    @details
    '''


    def __init__(self,AR, Tim_N, CH_A_PIN, CH_B_PIN):
        '''!@brief Constructs an encoder object
        @details
        @param
        @param
        @param
        '''
        self.Tim_N = Tim_N
        self.AR = AR
        self.CHA = Tim_N.channel(1, pin=CH_A_PIN, mode=Timer.ENC_AB)
        self.CHB = Tim_N.channel(2, pin=CH_B_PIN, mode=Timer.ENC_AB)
        self.delta = 0
        self.my_count = 0 #Tim_N.counter()
        self.position = 0
        self.old_pos = 0
        self.new_pos = 0
        self.current_counter_value = 0
        self.step = 0
        self.frame = 0
        self.h_AR = (AR+1)//2
        pass


    def update(self):
        '''!@brief Updates encoder position and delta
        @details
        '''
        self.step = time.ticks_diff(time.ticks_us(),self.frame)
        self.old_pos = self.my_count
        self.my_count = self.Tim_N.counter()
        self.new_pos = self.my_count
        self.delta = self.new_pos - self.old_pos
        if self.delta > self.h_AR:
            self.delta -= self.AR + 1
            self.position += self.delta
        elif self.delta < -self.h_AR:
            self.delta += self.AR+1
            self.position += self.delta
        else:
            self.position += self.delta
            pass
        self.frame = time.ticks_us()
        return


    def get_position(self):
        '''!@brief Gets the most recent encoder position
        @details
        @return
        '''
        return self.position*2*3.14/1440


    def get_omega(self):
        '''!@brief Gets the most recent encoder delta and calculates omega through it
        @details
        @return
        '''
        return self.delta*6.28/(1440*(self.step/1000000))


    def zero(self):
        '''!@brief Resets the encoder position to zero
        @details
        '''
        self.delta = 0
        self.my_count = self.Tim_N.counter()
        self.new_pos = 0
        self.position = 0
        pass
