class ClosedLoop:

    def __init__(self,Kp,Vref,Mode):
        #self.loopdedoo = loopdedoo
        self.Kp = Kp
        self.Vref = Vref
        self.Mode = Mode
        self.L = 0
        #self.Motor = Motor
        #self.Encoder = Encoder

    def set_Kp(self, kp):
        self.Kp = kp
        return
    def set_mode(self,mode):
        self.Mode = mode
        return
    def get_mode(self):
        return self.Mode
    def set_Velocity(self,vref):
        self.Vref = vref
        return
    def update(self, vmeas):
        if self.Mode == 'CL':
            self.L = self.Kp*(self.Vref - vmeas)
            #print(str(self.L))
            return self.L
        elif self.mode == 'OL':
            return self.Vref
        else:
            raise ValueError("OL or CL not selected properly")
    
    # def get_Velocity(self):
    #     return self.Velocity
