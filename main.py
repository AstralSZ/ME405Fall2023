##@file main.py 
# 
# This file contains the function that runs the scheduler and creates the shares required for the tasks.


import cotask
import pyb
from task_share import Share, Queue
from pyb import UART, repl_uart
from user_interface import user_interface
from Hardware import hardware
from Data_Transfer import Data_Transfer



if __name__ == "__main__":
    # Disable REPL on UART2
    ser = pyb.USB_VCP()
    uart = UART(2, 115200)
    pyb.usb_mode('VCP')
    # Create some shares to use for the task
    
    
    run_flag: Share = Share('h', name='Run Romi flag')
    
    
    # IMU flags
    eul_flag: Share = Share('h', name='Euler Angle flag')
    gyro_flag: Share = Share('h', name='Angular Acceleration flag')
    cal_stat_flag: Share = Share('h', name='Calibration status flag')
    cal_coeff_flag: Share = Share('h', name='Calibration Coefficients flag')
    change_mode: Share = Share('h', name='IMU Mode flag')

    

    
    
    # Create Objects
    user_int_obj = user_interface(ser, change_mode, cal_stat_flag, eul_flag, gyro_flag,cal_coeff_flag,run_flag)
    hardware_obj = hardware(change_mode, cal_stat_flag, eul_flag, gyro_flag,cal_coeff_flag,run_flag)
    task1 = cotask.Task(user_int_obj.run, name='User Interface', priority=2, period=100)
    task2 = cotask.Task(hardware_obj.run, name='Hardware', priority=3, period=15)

    # Append the newly created task to the task lista_fl
    cotask.task_list.append(task1)
    cotask.task_list.append(task2)

    # Run the scheduler
    while True:
        cotask.task_list.pri_sched()