##@file tracking.py
# @author Say Zhee & Will Barics

import math
import time

## @brief Interface with control loops
class Tracking:
    ## Tracking
    # @details 
    def __init__ (self):
        ## @brief Constructs a tracking object
        #
        # Initialzes important local variables
        #

        ## Radius of a wheel
        self.r = 1.358 #in

        ## Half of Romi wheel base length
        self.py = 5.512 #in

        ## Time step from numeric integration
        self.tstep = 0

        ## Velocity in x direction
        self.xdot = 0

        ## Velocity in y direction
        self.ydot = 0

        ## Reference time point
        self.frame = time.ticks_us()
        
        
    def calculate(self,omega_1,omega_2,x,y,theta):
        ## @brief Calculates the X and Y position 
        # @details Calculates the time step based on the difference in time between the last time the function was called.
        # Calculates xdot and ydot from the wheel speed from the encoder and theta from IMU. Finds 
        # @param omega_1: Angular velocity of left wheel in rad/s
        # @param omega_2: Angular velocity of right wheel in rad/s
        # @param x: Old x position
        # @param y: Old y poistion
        # @param theta: Angle romi is facing from the IMU's euler angles
        self.tstep = time.ticks_diff(time.ticks_us(),self.frame)/1000000
        self.xdot = (self.r/2)*(omega_1 + omega_2)*math.cos(theta)
        self.ydot = (self.r/2)*(omega_1 + omega_2)*math.sin(theta)
        self.frame = time.ticks_us()
        return x+(self.xdot*self.tstep), y+(self.ydot*self.tstep)
      
        
    # def truncate(number, digits) -> float:
    #     # Improve accuracy with floating point operations, to avoid truncate(16.4, 2) = 16.39 or truncate(-1.13, 2) = -1.12
    #     nbDecimals = len(str(number).split('.')[1]) 
    #     if nbDecimals <= digits:
    #         return number
    #     stepper = 10.0 ** digits
    #     return math.trunc(stepper * number) / stepper