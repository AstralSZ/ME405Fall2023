
## @file closedloop.py
#
# brief Interface with control loops




## Initializes closed loop object that returns a motor duty using proportional control
#
#  ClosedLoop class is to be used to create an closedloop object which counter intuitively can be used for closed or open loop control. 
#  It allows the user to update the constant of proportionality (Kp), the mode, the reference value. It allows the user to zero the error, get the mode,
#  and update the control logic with measured values.
    

class ClosedLoop:

    
    ## Constructs a closedloop object
    #
    # Initialzes the passed in parameters as well as important local variables
    #
    # @param Kp: Constant of proportionality
    # @param Vref: Reference value
    # @param Mode: 'Cl' for closed loop or 'OL' for open loop
    def __init__(self,Kp,Vref,Mode):
       
        self.Kp = Kp
        self.Vref = Vref
        self.Mode = Mode
        self.L = 0


    ## Sets proportional gain constant Kp
    #
    # Updates the local variable Kp with paramerter kp
    #
    # @param kp: Constant of Proportionality
    def set_Kp(self, kp):
        
        self.Kp = kp
        return
    
    
    
    
    def set_mode(self,mode):
        ##@brief Sets mode  
        #
        # Updates the local variable Mode with paramerter mode 
        #  
        # @param mode: 'Cl' for closed loop or 'OL' for open loop
    
        self.Mode = mode
        return
    def get_mode(self):
        ##@brief Gets mode  
        #
        # Returns value of mode. Either 'Cl' for closed loop or 'OL' for open loop
        #
        # @return Mode 'Cl' for closed loop or 'OL' for open loop
        
        return self.Mode
    def set_Velocity(self,vref):
        ##@brief Sets the reference value  
        #
        # Updates the reference value with the passed in parameter
        #
        # @param vref: Reference value for control
        
        self.Vref = vref
        return
    
    
    ## Zeros error 
    #
    # Sets the variable for error to zero.    
    def zero_L(self):
        
        self.L = 0
        return
    
    
    ## Sets the reference value  
    #
    # If in closed loop mode, takes the difference between the reference value and the measured value
    # and multiplies it by Kp to find the error and returns this error.
    # If in open loop mode, returns the reference value.
    #
    # @param vmeas: Measured value
    # @return If OL returns reference value. If CL returns error. 
    def update(self, vmeas):
        
        if self.Mode == 'CL':
            self.L = self.Kp*(self.Vref - vmeas)
            return self.L
        elif self.mode == 'OL':
            return self.Vref
        else:
            raise ValueError("OL or CL not selected properly")

