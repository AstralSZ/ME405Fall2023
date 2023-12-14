"""
@file encoder.py
@author: Say Zhee & Will Barics
"""
from pyb import Pin, Timer
import time

class Encoder:
    '''!@class Encoder
    @brief Interface with Romi's encoders
    @details Encoder class is to be used to create an Encoder object for each motor with an encoder. 
    It allows the user to update the encoder, read the wheel's position in radians, read the wheel's speed in rad/s, and zero the encoder
    '''


    def __init__(self,AR, Tim_N, CH_A_PIN, CH_B_PIN):
        '''!@brief Constructs an encoder object
        @details: Initialzes the passed in parameters as well as important local variables
        @param AR: Auto-Reload value for the timer
        @param Tim_N: Timer object created outside of the class
        @param CH_A_PIN: Encoder Channel A Pin
        @param CH_B_PIN: Encoder Channel B Pin
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
        @details measures the step between last time called. Updates old position with old count variable, 
        updates count varible with timer, updates new position with new count varible,
        updates delta with the difference between new count and old count. Implements logic to add or subtract auto-reload
        from delta depending on if delta is greater than or less than the auto reload. Updates the position with the new delta.
        Finaly updates the frame with the new ticks count in microseconds.
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
        @details Encoder position multiplied by conversion constant to convert to rad
        @return Position of Romi's wheel in rad
        '''
        return self.position*2*3.14/1440


    def get_omega(self):
        '''!@brief Gets the most recent encoder delta and calculates omega through it
        @details Delta multiplied by conversion constant to convert to rad/s
        @return Omega of Romi's wheel in rad/s
        '''
        return self.delta*6.28/(1440*(self.step/1000000))


    def zero(self):
        '''!@brief Resets the encoder position to zero
        @details Assigns 0 to delta, postion, and new position (used in update). Sets count variable to current timer value
        '''
        self.delta = 0
        self.my_count = self.Tim_N.counter()
        self.new_pos = 0
        self.position = 0
        pass
