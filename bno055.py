## @file bno055.py
# This file contains the class to interface with the BNO055 9-DOF IMU.
#
# This file only works for the BNO055 as registers are written and 
# read based on the documentation of the device. The user is able to
# change the mode of the IMU, get the calibration status and coefficients
# of the device, and save a copy or write coefficients for easier calibration
# into the device.


from pyb import I2C
import struct
import os
import time


## Interfaces with the BNO055
# 
# This class enables the user to easily communicate with the IMU
# and obtain information of the Euler angles and angular acceleration.
# It also enables simple configuration of the device

class BNO055:
    ## Initialize a BNO055 object so it can be read and written to.
    # 
    # This method initializes the BNO055 address with the controller and peripheral address
    # of the IMU.
    # 
    # @param I2C_object The controller of the IMU
    # @param Prep_Addr The peripheral address of the IMU

    def __init__(self,I2C_object,Prep_Addr):
        self.I2C_object = I2C_object
        self.Prep_Addr = Prep_Addr
        
        ## The angular acceleration in the x direction to be returned by the IMU
        self.acc_x = 0
        
        ## The angular acceleration in the y direction to be returned by the IMU
        self.acc_y = 0
        
        ## The angular acceleration in the z direction to be returned by the IMU
        self.acc_z = 0
        
        ## The euler angle in the x direction to be returned by the IMU
        self.eul_x = 0
        
        ## The euler angle in the x direction to be returned by the IMU
        self.eul_y = 0
        
        ## The euler angle in the x direction to be returned by the IMU
        self.eul_z = 0
        
        ## Flag to check if the system has been calibrated
        self.sys_status = 0
        
        ## Tuple to hold calibration coefficients
        self.calib_coeffs = ()
        
        ## Flag for checking whether the current mode is in Configuration        
        self.config = 0
    
    ## This method changes the mode of the BNO055.
    #  User is able to change the mode of the IMU into one of
    #  5 fusion modes for the BNO055 or switch into configuration mode.
    #
    #  @param mode String of the mode the user wants to switch to
    def change_mode(self, mode):
        # IMU, COMPASS, M4G, NDOF_FMC_OFF, NDOF
        #[OPR_MODE] 1000,1001,1010,1011,1100
        # 0x3D write ^^^^ to select mode
        #[UNIT_SEL] = 00000110 %Windows,C,Rad,R/s,m/s^2
        if mode == 'IMU':
            self.I2C_object.mem_write(0x08,self.Prep_Addr,0x3D)
            self.config = 0
            print("Switched to IMU mode")
        elif mode == 'COMPASS':
            self.I2C_object.mem_write(0x09,self.Prep_Addr,0x3D)
            self.config = 0
            print("Switched to COMPASS mode")
        elif mode == 'M4G':
            self.I2C_object.mem_write(0x0A,self.Prep_Addr,0x3D)
            self.config = 0
            print("Switched to M4G mode")
        elif mode == 'NDOF_FMC_OFF':
            self.I2C_object.mem_write(0x0B,self.Prep_Addr,0x3D)
            self.config = 0
            print("Switched to NDOF_FMC_OFF mode")
        elif mode == 'NDOF':
            self.I2C_object.mem_write(0x0C,self.Prep_Addr,0x3D)
            self.config = 0
            print("Switched to NDOF mode")
        elif mode == 'CONFIG':
            self.I2C_object.mem_write(0x00,self.Prep_Addr,0x3D)
            self.config = 1
            print("Switched to config mode")
        else:
            print('Invalid mode entered')
        return 
    
    
    ## This method initializes the BNO055 and writes calibration coeffcients
    #  if a file exists.
    #
    #  The axis are remapped to mirror the axis to be used for dead reckoning and 
    #  checks whether a file of calibration exists and writes them to the IMU if so.
    #
    #  @param mode String of the mode the user wants to switch to    
    def run(self): # Remap axis and write file to calibration coefficients if it exists
        if self.config == 1:
            self.I2C_object.mem_write(0x21,self.Prep_Addr,0x41)
            self.I2C_object.mem_write(0x02,self.Prep_Addr,0x42)
            print('Axis remapped')
            if "IMU_cal_coeffs.txt" in os.listdir():
                with open('IMU_cal_coeffs.txt','r') as file:
                    line = file.readline()
                    byte_list = line.strip()
                    byte_list = byte_list.split(",")
                    if len(byte_list) == 22:
                        for i in range(22):
                            self.I2C_object.mem_write(int(byte_list[i]),self.Prep_Addr,0x55+i)
                        print("Finished configuration with file")
                        buf = bytearray(22)
                        self.I2C_object.mem_read(buf,self.Prep_Addr,0x55)
                        self.calib_coeff = struct.unpack('>22b',buf)
                        print(self.calib_coeff)
                    else:
                        print("Incorrect file format")
            else:
                print("No IMU file detected please go through calibration process")
        else:
            print("Switch to config mode first before running")
        return

    ## This method enables the user to view the calibration status bits.
    #  
    #  The components that are checked include the system, gyroscope, accelerometer, and magnetometer.
    #  If '11' is returned that means that the corresponding component is calibrated and '00' means
    #  that the components is uncalibrated.
    #
    #  @returns A string that represents the calibration status of each component of the BNO055. 
    def get_cs(self):
        # [CALIB_STAT] 0x35
        # SYS 7,6 | GYR 5,4 | ACC 3,2 | MAG 1,0 
        buf = bytearray(1)
        self.I2C_object.mem_read(buf,self.Prep_Addr,0x35)
        binary_representation = bin(buf[0] & 0xFF)[2:]
        binary = '0' * (8 - len(binary_representation)) + binary_representation
        if binary[6:8] == '11':
            self.sys_status = 1
        if binary[4:6] == '00':
            print("Leave Romi standing still until calibrated")
        if binary[2:4] == '00':
            print("Rotate Romi at 45 degree increments")
        if binary[:2] == '00':
            #print('Lift Romi upside down and create figure eights')
            pass
        self.I2C_object.mem_write(0x0C,self.Prep_Addr,0x3D)
        return ('Sys :' + binary[6:8] + ',' + 'Gyr: ' + binary[4:6] + ',' + 'Acc: ' + binary[2:4] + ',' + 'Mag: ' + binary[:2]) # SYS,GYR,ACC,MAG
        
    
    ## This method enables the user to view the calibration coefficients.
    #
    #  Reads the register cotaining the 22 calibiration coeffcients and returns them as a tuple in hex form.
    #  If no previous calibration file exists, writes the current calibration coeffcients into a txt file
    #  allowing them to be used in the future.
    #
    # @returns A tuple containing 22 hex numbers corresponding to the value of the calibration coefficients.
    def get_cc(self):
        buf = bytearray(22)
        self.I2C_object.mem_read(buf,self.Prep_Addr,0x55)
        self.calib_coeff = struct.unpack('<22b',buf)
        if not "IMU_cal_coeffs.txt" in os.listdir() and self.sys_status == 1:
            with open('IMU_cal_coeffs.txt' , 'w') as file:
                print("Writing coefficients to file")
                for i in range((len(self.calib_coeff)-1)):
                    file.write(str(hex(self.calib_coeff[i])) + ',')
                file.write(str(hex(self.calib_coeff[21])))
        return self.calib_coeff
             
    ## This method returns the Euler Angles calculated by the IMU.
    # 
    #  Reads the register containing the Euler Angles calculated by the BNO055 enabling
    #  them to be used and read. Gives the euler angles in units of degrees of the entire 
    #  coordinate system.
    #
    #  @returns A tuple containing the value of the Euler Angles in the x,y,z direction in units of degrees
    def get_ea(self): # Euler Angles
        if self.sys_status != 1:
            #print('Warning system not fully calibrated')
            pass
        buf = bytearray(6)
        self.I2C_object.mem_read(buf,self.Prep_Addr,0x1A)
        self.eul_x, self.eul_y, self.eul_z = struct.unpack('<hhh',buf)
        return self.eul_x/16, self.eul_y/16, self.eul_z/16
        
    ## This method returns the angular velocities read by teh IMU.
    # 
    #  Reads the reister containing the information of angular velocities found by the BNO055. 
    #  Gives the angular velocoites in units of degrees/s
    #
    # @returns A tuple containing the value of the angular velocities in the x,y,z direction
    # in units of degrees/second
    def get_av(self): # Angular Velocities
        if self.sys_status != 1:
            print('Warning system not fully calibrated')
        buf = bytearray(6)
        self.I2C_object.mem_read(buf,self.Prep_Addr,0x14)
        self.acc_x, self.acc_y, self.acc_z = struct.unpack('<hhh',buf)
        return self.acc_x/16, self.acc_y/16, self.acc_z/16