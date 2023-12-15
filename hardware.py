## @file hardware.py
#
# This file contains the class for the hardware that is used as one of the tasks used for the Romi robot.
# Hardware handles the robot's sensors and motor control
# 
from pyb import UART, repl_uart,Timer,Pin,I2C,ADC
from task_share import Queue
import cotask
import time
import math
from encoder import Encoder
#from Collector import Collector
from romi_Motor import Rom
from closedLoop import ClosedLoop
from bno055 import BNO055
from tracking import Tracking
from hcsr04 import HCSR04


# Still left to do 
# 1) put if statements to determine which alphabetical command has been pressed
# 2) more work is needed on the collecting data states ie collecting data for 30 sec as well as transporting the data to the appropriate share/queue?

class hardware:
    '''! A class implementing a data transfer task
    '''

    def __init__(self,change_mode, cal_stat_flag, eul_flag, gyro_flag,cal_coeff_flag,run_flag):
        self.state = 0
        self.change_mode = change_mode
        self.cal_stat_flag = cal_stat_flag
        self.eul_flag = eul_flag
        self.gyro_flag = gyro_flag
        self.cal_coeff_flag = cal_coeff_flag
        self.run_flag = run_flag
        self.base = 20
        self.mode = 0
        self.part = 0
        self.wall = 0
        self.vmeas1 = 0
        self.vmeas2 = 0
        # Create a timer object to use for motor control
        tim_A = Timer(3, freq = 20000)
        tim_B = Timer(4, freq = 20000)
        self.CL_1 = ClosedLoop(Kp = 15,Vref = 10,Mode = 'CL')
        self.CL_2 = ClosedLoop(Kp = 15,Vref = 10,Mode = 'CL')
        self.CL_3 = ClosedLoop(Kp = 15,Vref = 10,Mode = 'CL')
        self.CL_4 = ClosedLoop(Kp = 15,Vref = 10,Mode = 'CL')
    
        # Create an L6206 driver object. You will need to modify the code to facilitate
        # passing in the pins and timer objects needed to run the motors.
        self.mot_A = Rom(tim_A, Pin.cpu.B4, Pin.cpu.B5, Pin.cpu.A10)
        self.mot_B = Rom(tim_B, Pin.cpu.B6, Pin.cpu.B7, Pin.cpu.C1)
        # Enable the L6206 driver
        self.mot_A.enable()
        self.mot_B.enable()
        # Set the duty cycle of the first L6206 channel to 40 percent
        self.mot_A.set_duty(0)
        self.mot_B.set_duty(0)

        tim_4 = Timer(2, period = 65535, prescaler = 0)
        tim_2 = Timer(1, period = 65535, prescaler = 0)
        self.encoder_1 = Encoder(AR = 65535, Tim_N=tim_2, CH_A_PIN=Pin.cpu.A8,CH_B_PIN=Pin.cpu.A9)
        self.encoder_2 = Encoder(AR = 65535, Tim_N=tim_4, CH_A_PIN=Pin.cpu.A0,CH_B_PIN=Pin.cpu.A1)
        self.CL = ClosedLoop(Kp = .019,Vref = 2000,Mode = 'CL')
        # Create a timer object to use for motor control
        # base 25 Kp .023
        self.adc1=ADC(Pin(Pin.cpu.A5)) #M
        self.adc2=ADC(Pin(Pin.cpu.B0)) #R
        self.adc3=ADC(Pin(Pin.cpu.C0)) #L
        self.adc4=ADC(Pin(Pin.cpu.A4)) #FR
        self.adc5=ADC(Pin(Pin.cpu.A6)) #FL
        self.adc6=ADC(Pin(Pin.cpu.A7)) #FFR
        self.adc7=ADC(Pin(Pin.cpu.C4)) #FFL

        self.sensor = HCSR04(trigger_pin=Pin.cpu.B10, echo_pin=Pin.cpu.B3, echo_timeout_us=10000)
        self.track = Tracking()

        
        
        self.i2c = I2C(1, I2C.CONTROLLER)              # create and init as a controller
        self.i2c.init(I2C.CONTROLLER, baudrate=115200) # init as a controller
        self.BNO055 = BNO055(self.i2c,0x28)
        
        self.BNO055.change_mode('CONFIG')
        time.sleep(0.1)
        self.BNO055.run()
        time.sleep(0.1)
        self.BNO055.change_mode('NDOF')

        self.start_pos = 0
        self.pos = 0


   
    def run(self):
        while True:
            if self.state == 0:
                self.state = 1
            elif self.state == 1:
                self.encoder_1.update()
                self.encoder_2.update()
                if self.cal_stat_flag.get() == 1:
                    self.cal_stat_flag.put(0)
                    self.state = 2
                elif self.eul_flag.get() == 1:
                    self.eul_flag.put(0)
                    self.state = 3
                elif self.cal_coeff_flag.get() == 1:
                    self.cal_coeff_flag.put(0)
                    self.state = 4
                elif self.gyro_flag.get() == 1:
                    self.gyro_flag.put(0)
                    self.state = 5
                elif self.change_mode.get() !=  0:
                    self.state = 6
                elif self.cal_stat_flag.get() == 2:
                    self.cal_stat_flag.put(0)
                    self.state = 7
                elif self.run_flag.get() == 1:
                    print("Starting Romi")
                    self.mode = 1
                    self.x = 0
                    self.y = 0
                    self.theta = 0
                    self.ref_angle,_,_ = self.BNO055.get_ea()
                    self.encoder_1.zero()
                    self.encoder_2.zero()
                    self.state = 8
                else:
                    pass
            elif self.state == 2:
                print(self.BNO055.get_cs())
                self.state = 1
            elif self.state == 3:
                print(self.BNO055.get_ea())
                self.state = 1
            elif self.state == 4:
                print(self.BNO055.get_cc())
                self.state = 1
            elif self.state == 5:
                print(self.BNO055.get_av())
                self.state = 1
            elif self.state == 6:
                if self.change_mode.get() == 1:
                    self.BNO055.change_mode('IMU')
                elif self.change_mode.get() == 2:
                    self.BNO055.change_mode('COMPASS')
                elif self.change_mode.get() == 3:
                    self.BNO055.change_mode('M4G')
                elif self.change_mode.get() == 4:
                    self.BNO055.change_mode('NDOF_FMC_OFF')
                elif self.change_mode.get() == 5:
                    self.BNO055.change_mode('NDOF')
                elif self.change_mode.get() == 6:
                    self.BNO055.change_mode('CONFIG')
                self.change_mode.put(0)
                self.state = 1
            elif self.state == 7:
                self.BNO055.run()
                self.state = 1
            elif self.state == 8:
                self.encoder_1.update()
                self.encoder_2.update()
                self.heading,_,_ = self.BNO055.get_ea()
                self.theta = (self.heading-self.ref_angle)*math.pi/180
                if self.theta < 0:
                   self.theta = self.theta + (360*math.pi/180)
                self.x,self.y = self.track.calculate(self.encoder_1.get_omega(),self.encoder_2.get_omega(),self.x,self.y,self.theta)
                #print('X: ' +  str(self.x),'Y: ' + str(self.y), 'Theta: ' +str(self.theta))
                #print(self.sensor.distance_cm())
                if self.run_flag.get() == 0:
                    self.state = 1
                    self.mode = 1
                    self.part = 0
                    self.wall = 0
                    self.mot_A.set_duty(0)
                    self.mot_B.set_duty(0)
                    print("Stopping Romi")                
                elif self.mode == 1:
                    if self.sensor.distance_cm() < 4 and self.wall == 0:
                        self.start_pos = 0
                        print('mode 2')
                        self.mode = 2
                        self.part = 1
                    elif self.adc1.read() > 1000 and self.adc2.read() > 1000 and self.adc3.read() > 1000 and self.adc3.read() > 1000 and self.adc4.read() > 1000 and self.adc6.read() > 1000 and self.adc7.read() > 1000 and self.wall == 1:
                        self.frame = time.ticks_ms()
                        self.mode = 4
                    elif self.adc1.read() < 1200 and self.adc2.read() < 1200 and self.adc3.read() < 1200 and self.adc4.read() < 1200 and self.adc5.read() < 1200 and self.adc6.read() < 1000 and self.adc7.read() < 1000:
                        self.mot_A.set_duty(self.base)
                        self.mot_B.set_duty(self.base)
                    elif self.adc1.read() < 1700:
                        self.start_pos = 0
                        self.val = abs(self.CL.update(self.adc5.read()))
                        if self.adc2.read() > 1500:
                            print('R')
                            self.mot_A.set_duty(self.base+self.val)
                            self.mot_B.set_duty(self.base-self.val)
                            print(self.val)
                        elif self.adc3.read() > 1500:
                            print('L')
                            self.mot_A.set_duty(self.base-self.val)
                            self.mot_B.set_duty(self.base+self.val)
                            print(self.val)
                        elif self.adc4.read() > 1500:
                            print('FR')
                            self.mot_A.set_duty(self.base+self.val)
                            self.mot_B.set_duty(self.base-self.val)
                            print(self.val)
                        elif self.adc5.read() > 1500:
                            print('FL')
                            self.mot_A.set_duty(self.base-self.val)
                            self.mot_B.set_duty(self.base+self.val)
                            print(self.val)
                        elif self.adc6.read() > 1700:
                            self.mot_A.set_duty(self.base+self.val)
                            self.mot_B.set_duty(self.base-self.val)
                        elif self.adc7.read() > 1700:
                            self.mot_A.set_duty(self.base-self.val)
                            self.mot_B.set_duty(self.base+self.val)
                    else:
                        self.start_pos = 0
                        self.mot_A.set_duty(self.base)
                        self.mot_B.set_duty(self.base)
                elif self.mode == 2: # Wall detected, drive around
                    self.wall = 1
                    if self.part == 1:
                        if abs(self.theta-(math.pi/2)) > 0.1:
                            #print("Turning")
                            self.mot_A.set_duty(10)
                            self.mot_B.set_duty(-10)
                        else:
                            self.frame = time.ticks_ms()
                            self.part = 2
                    elif self.part == 2:
                        if time.ticks_diff(time.ticks_ms(),self.frame) < 800:
                            self.vmeas1 = abs(self.encoder_1.get_omega())
                            self.vmeas2 = abs(self.encoder_2.get_omega())
                            print(self.vmeas1 , self.vmeas2)
                            # self.mot_A.set_duty(20)
                            # self.mot_B.set_duty(20)
                            self.mot_A.set_duty(self.CL_1.update(self.vmeas1))
                            self.mot_B.set_duty(self.CL_2.update(self.vmeas2))
                        else:
                            self.part = 3
                    elif self.part == 3:
                        if abs(self.theta-0) > 0.05:
                            self.mot_A.set_duty(-10)
                            self.mot_B.set_duty(10)
                        else:
                            self.CL_1.zero_L()
                            self.CL_2.zero_L()
                            self.vmeas1 = 0
                            self.vmeas2 = 0
                            self.frame = time.ticks_ms()
                            self.part = 4
                    elif self.part == 4:
                        if time.ticks_diff(time.ticks_ms(),self.frame) < 1700:
                            # self.mot_A.set_duty(20)
                            # self.mot_B.set_duty(20)
                            self.vmeas1 = abs(self.encoder_1.get_omega())
                            self.vmeas2 = abs(self.encoder_2.get_omega())
                            print(self.vmeas1 , self.vmeas2)
                            # self.mot_A.set_duty(20)
                            # self.mot_B.set_duty(20)
                            self.mot_A.set_duty(self.CL_1.update(self.vmeas1))
                            self.mot_B.set_duty(self.CL_2.update(self.vmeas2))
                        else:
                            self.part = 5
                    elif self.part == 5:
                        if abs(self.theta-1.5*(math.pi)) > 0.05:
                            self.mot_A.set_duty(-10)
                            self.mot_B.set_duty(10)
                        else:
                            self.frame = time.ticks_ms()
                            self.part = 6    
                    elif self.part == 6:
                        #if time.ticks_diff(time.ticks_ms(),self.frame) < 1500:
                        self.vmeas1 = abs(self.encoder_1.get_omega())
                        self.vmeas2 = abs(self.encoder_2.get_omega())
                        print(self.vmeas1 , self.vmeas2)
                        # self.mot_A.set_duty(20)
                        # self.mot_B.set_duty(20)
                        self.mot_A.set_duty(self.CL_1.update(self.vmeas1))
                        self.mot_B.set_duty(self.CL_2.update(self.vmeas2))
                        if self.adc1.read() > 1000 and self.adc2.read() > 1000 and self.adc3.read() > 1000 and self.adc3.read() > 1000 and self.adc4.read() > 1000:# and self.adc6.read() > 1000 and self.adc7.read() > 1000:
                            self.part = 7
                            self.frame = time.ticks_ms()
                    elif self.part == 7:
                        if time.ticks_diff(time.ticks_ms(),self.frame) < 200:
                            self.vmeas1 = abs(self.encoder_1.get_omega())
                            self.vmeas2 = abs(self.encoder_2.get_omega())
                            print(self.vmeas1 , self.vmeas2)
                            self.mot_A.set_duty(self.CL_1.update(self.vmeas1))
                            self.mot_B.set_duty(self.CL_2.update(self.vmeas2))
                        else:
                            self.part =8
                    elif self.part == 8:
                        if abs(self.theta-0) > 0.05:
                            self.mot_A.set_duty(10)
                            self.mot_B.set_duty(-10)
                        else:
                            self.mode = 1
                            self.part = 1  
                elif self.mode == 3: # Back to start
                    self.pos = 0
                    self.start_pos = 0
                    self.wall = 0
                    if self.part == 1:
                        if self.x > 0:
                            self.part = 3
                        else:
                            print(self.theta)
                            if abs(self.theta-(3*math.pi/2)) > 0.05:
                                self.mot_A.set_duty(-10)
                                self.mot_B.set_duty(10)
                            else:
                                self.part = 2
                    elif self.part == 2:
                        print(self.y)
                        if self.y <= -1:
                            self.vmeas1 = abs(self.encoder_1.get_omega())
                            self.vmeas2 = abs(self.encoder_2.get_omega())
                            print(self.vmeas1 , self.vmeas2)
                            # self.mot_A.set_duty(20)
                            # self.mot_B.set_duty(20)
                            self.mot_A.set_duty(self.CL_1.update(self.vmeas1))
                            self.mot_B.set_duty(self.CL_2.update(self.vmeas2))
                        else:
                            self.part = 3
                    elif self.part == 3:
                        print('Turning')
                        print(self.theta)
                        if self.x > 0:
                            if abs(self.theta-0) > 0.08:
                                self.mot_A.set_duty(10)
                                self.mot_B.set_duty(-10)
                                self.CL_3.zero_L()
                                self.CL_4.zero_L()
                                self.vmeas1 = 0
                                self.vmeas2 = 0
                            else:
                                self.part = 4
                        elif self.x < 0: 
                             if abs(self.theta-(math.pi)) > 0.1:
                                self.mot_A.set_duty(-10)
                                self.mot_B.set_duty(10)
                             else:
                                self.CL_3.zero_L()
                                self.CL_4.zero_L()
                                self.vmeas1 = 0
                                self.vmeas2 = 0
                                self.part = 4
                    elif self.part == 4:
                        #print(self.x)
                        self.vmeas1 = abs(self.encoder_1.get_omega())
                        self.vmeas2 = abs(self.encoder_2.get_omega())
                        if abs(self.x) > 0.1:
                            # self.mot_A.set_duty(20)
                            # self.mot_B.set_duty(20)
                            #print(self.CL_1.update(vmeas1), self.CL_2.update(vmeas2))
                            self.mot_A.set_duty(self.CL_3.update(self.vmeas1))
                            self.mot_B.set_duty(self.CL_4.update(self.vmeas2))
                            print(self.vmeas1 , self.vmeas2)
                        else:
                            self.part = 5  
                    elif self.part == 5:
                        self.mot_A.set_duty(0)
                        self.mot_B.set_duty(0)
                        #self.state = 1
                        self.part = 0
                        self.mode = 0
                    else: 
                        self.mot_A.set_duty(0)
                        self.mot_B.set_duty(0)
                        self.state = 1       
                        self.part = 0
                        self.mode = 1
                elif self.mode == 4:
                    if time.ticks_diff(time.ticks_ms(),self.frame) < 700:
                            self.vmeas1 = abs(self.encoder_1.get_omega())
                            self.vmeas2 = abs(self.encoder_2.get_omega())
                            print(self.vmeas1 , self.vmeas2)
                            self.mot_A.set_duty(self.CL_1.update(self.vmeas1))
                            self.mot_B.set_duty(self.CL_2.update(self.vmeas2))
                    else:
                        self.mode = 3
                    
            else:
                raise ValueError("Hardware task state has exceeded bounds")
            yield 
        