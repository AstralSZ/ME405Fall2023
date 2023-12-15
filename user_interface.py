## @file user_interface.py
# 
# This file contains the class for the user interface that is used as one of the tasks
# used for the Romi robot.


import cotask
import pyb
from task_share import Share
from pyb import UART, repl_uart,Pin,ExtInt
global run
run = 0


## Initialize a user interface object to be used by cotask
#
# User interface is used a finite state machine that waits for key presses or button presses
# to set flags for the hardware task to act
class user_interface:
    
    ## Constructs an user_interface object to be called by cotask
    # 
    #  Requires the necessary shares and serial connection objects to initialize the user_interface 
    #
    #  @param ser Serial connection of microcontroller
    #  @param change_mode Flag to change mode of the IMU
    #  @param cal_stat_flag Flag to view the calibration status of the IMU
    #  @param eul_flag Flag to view the euler angles calculated by the IMU
    #  @param gyro_flag Flag to view the angular velocities read by the IMU
    #  @param cal_coeff_flag Flag to view current calibration coefficients in the IMU
    #  @param run_flag Flag to signify Romi to start navigating
    
    def __init__(self, ser, change_mode, cal_stat_flag, eul_flag, gyro_flag,cal_coeff_flag,run_flag):
        ## Serial port object
        self.ser = ser
        
        ## Calibration Status flag
        self.cal_stat_flag = cal_stat_flag
        
        ## Euler Angle flag
        self.eul_flag = eul_flag
        
        ## Gyroscope Euler Angle flag
        self.gyro_flag = gyro_flag
        
        ## Calibration Coefficients flag
        self.cal_coeff_flag = cal_coeff_flag
        
        ## Change IMU mode flags
        self.change_mode = change_mode
        
        ## Run Romi flag
        self.run_flag = run_flag
        
        ## State variable
        self._state = 0
        
        ## List to hold character input
        self.char_buf = []
        
        ## IMU command flag
        self.IMU_com = 0
        
        ## Input number flag
        self.num_set = 0
        
        ## Setting IMU Mode flag
        self.IMUset_mode = 0
        
        ## Run Romi flag
        self.buf = 0
        
        ## Blue button object
        self.button_int = ExtInt(Pin.cpu.C13, ExtInt.IRQ_FALLING,Pin.PULL_NONE, callback)

    ## Runs the user_interface task as a finite state machine
    #
    #  Includes 6 seperate states with state 1 being the hub state
    #  state 2 being the command selection state, state 3 as the character confirmation state,
    #  state 4 as the digit entering state, state 5 is the backspace, and state 6 is the set 
    #  run flag state

    def run(self):
        while True:
            if self._state == 0:
                self._state = 1
            if self._state == 1:  # Wait for character input
                if self.buf != run:
                    self.buf = run
                    self._state = 6
                elif self.ser.any():
                    charIn = self.ser.read(1).decode()
                    if charIn == '\x7f' and len(self.char_buf) != 0: # Backspace state
                        self._state = 5
                    elif charIn == '\n' or charIn == '\r': # Confirmation/Enter state
                        self._state = 3
                    elif self.IMU_com == 0 and len(self.char_buf) == 0 and self.num_set != 1: # Command choose state
                        self._state = 2
                    elif self.num_set == 1: # Digit entering state
                        print("Go to 4")
                        self._state = 4
                    else:       # Invalid entry
                        print("Passed")
                        pass
                else:
                    pass
            elif self._state == 2: # Command Handler
                if charIn == 'q' or charIn == 'Q' or  charIn == 'i' or charIn == 'I' or charIn == 'w' or charIn == 'W' or charIn == 'n' or charIn == 'r':
                    self.char_buf.append(charIn)
                    print(charIn)
                    self.IMU_com = 1  
                    self._state = 1
                else:
                    self._state = 1
            elif self._state == 3:  # Enter key handler
                if not self.char_buf:
                        print("|No command entered")
                        self._state = 1  
                elif self.IMUset_mode == 1:
                    if self.char_buf[0] == '1':
                        self.change_mode.put(1)
                        self.IMUset_mode = 0
                        self._state = 1
                    elif self.char_buf[0] == '2':
                        self.change_mode.put(2)
                        self.IMUset_mode = 0
                        self._state = 1
                    elif self.char_buf[0] == '3':
                        self.change_mode.put(3)
                        self.IMUset_mode = 0
                        self._state = 1
                    elif self.char_buf[0] == '4':
                        self.change_mode.put(4)
                        self.IMUset_mode = 0
                        self._state = 1
                    elif self.char_buf[0] == '5':
                        self.change_mode.put(5)
                        self.IMUset_mode = 0
                        self._state = 1       
                    elif self.char_buf[0] == '6':
                        self.change_mode.put(6)
                        self.IMUset_mode = 0
                        self._state = 1                           
                    self.IMU_com = 0
                    self.char_buf.clear()
                    self.num_set = 0
                elif self.IMU_com == 1:
                    if self.char_buf[0] == 'q':
                        self.cal_stat_flag.put(1)
                    elif self.char_buf[0] == 'Q':
                        self.cal_coeff_flag.put(1)
                    elif self.char_buf[0] == 'w':
                        self.eul_flag.put(1)
                    elif self.char_buf[0] == 'W':
                        self.gyro_flag.put(1)
                    elif self.char_buf[0] == 'i':
                        self.num_set = 1
                        self.IMUset_mode = 1
                        self.char_buf.clear()
                    elif self.char_buf[0] == 'I': # Initialize IMU
                        self.cal_stat_flag.put(2)
                    self._state = 1
                    self.char_buf.clear()
                    self.IMU_com = 0
            elif self._state == 4: # Number handler
                if len(self.char_buf) == 0:
                    if charIn == '-':
                        self.char_buf.append(charIn)
                        self.ser.write('|'+charIn+'\r\n')
                        print(self.char_buf)
                        self._state = 1
                    elif charIn.isdigit():
                        self.char_buf.append(charIn)
                        self.ser.write('|'+charIn+'\r\n')
                        print(self.char_buf)
                        self._state = 1
                elif charIn.isdigit():
                    self.char_buf.append(charIn)
                    self.ser.write('|'+charIn+'\r\n')
                    print(self.char_buf)
                    self._state = 1
                else:
                    self._state = 1
            elif self._state == 5: # Backspace Handler
                if self.num_set == 1:
                    self.uart.write('|Digit deleted\r\n')
                    self.char_buf.pop()
                    self._state =1
                elif self.IMU_com == 1:
                    print('Choose another command')
                    self.char_buf.clear()
                    self.motor_command = 0
                    self.IMU_com = 0
                    self._state = 1
                else:
                    self._state = 1
            elif self._state == 6:
                if self.buf == 0:
                    self.run_flag.put(0)
                else:
                    self.run_flag.put(1)
                self._state = 1
            yield self._state
            

def callback(line):
    global run
    if run == 1:
        run = 0
    else:
        run = 1
        
        
    
