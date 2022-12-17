#   LadyBug Technologies LLC August 3, 2021
#
#   Python demo v1.2.3 for LB[4|5|6]xx family of RF power sensors using old or new driver.
#
#   Please install sensor applications first to register the driver with the operating system.
#
#   Simple example of how to load the needed dll, select a sensor and make a measurement.
#
#   Tested on:
# python 64            3.9.6     3.8.10     3.7.9     3.6.6   
# python 32            3.9.6     3.8.10     3.7.9     3.6.6   
# =========================================================   
# New driver:
# Win 10 Pro 64/64     YES       YES        YES       YES     
# Win 10 Pro 64/32     YES       YES        ERROR     ERROR   
# Win 10 Pro 32/32     YES       YES        ERROR     ERROR   
# 
# Old driver:
# Win 10 Pro 64/64     YES       YES        YES       YES     
# Win 10 Pro 64/32     N/A       N/A        N/A       N/A     
# Win 10 Pro 32/32     N/A       N/A        N/A       N/A     
#
#
#
#
#  
#  
#   Required directory structure:
#  
#   -Python_TestHarness
#       Python_TestHarness(v1.2.3).py (This file)
#       Read Me
#       -DLLs
#           -old
#               -x64
#                   LB_API2.dll (Aug 7, 2011)
#               -x86
#                   LB_API2.dll (Feb 25, 2010)
#           -new
#               -x64 (Oct 8, 2021)(in file manager, right click->properties, select Details tab to show bitness of x64)
#                   LB_API2.dll
#                   LBUSBDRVD.dll
#               -x86 (Oct 8, 2021)(in file manager, right click->properties, select Details tab to show bitness of x86, 32-bit)
#                   LB_API2.dll
#                   LBUSBDRVD.dll
#  
#

import ctypes
import time
import os
import sys
import platform
from ctypes import *

def configure_windows():
    """
        Set paths to DLLs
    """

#    print("{0} bit".format((ctypes.sizeof(ctypes.c_voidp))*8))
#    print(platform.architecture()[0])
#    print(platform.architecture()[1])
#    print(platform.machine())
#    print(os.environ)
    

    try:  # to use Python 3.8's DLL handling
        os.add_dll_directory(os.getcwd()+ "/DLLs/new/x86/")
        os.add_dll_directory(os.getcwd()+ "/DLLs/old/x86/")
        os.add_dll_directory(os.getcwd()+ "/DLLs/new/x64/")
        os.add_dll_directory(os.getcwd()+ "/DLLs/old/x64/")
        print("Assuming Python 3.8+")
    except AttributeError:  # <3.8, use PATH
        os.environ['PATH']    += os.pathsep + os.getcwd() + "\\DLLs\\new\\x86\\"
        os.environ['PATH']    += os.pathsep + os.getcwd() + "\\DLLs\\old\\x86\\"
        os.environ['PATH']    += os.pathsep + os.getcwd() + "\\DLLs\\new\\x64\\"
        os.environ['PATH']    += os.pathsep + os.getcwd() + "\\DLLs\\old\\x64\\"
        os.environ['PATHEXT'] += os.pathsep + ".DLL"
        print("Assuming Python <3.8")
        




#========================================================================
#                        CHECK FOR DLLs
#========================================================================
#print("Current working directory = " + os.getcwd())
if not os.path.isfile("./DLLs/new/x64/LB_API2.dll"):
    print("./DLLs/new/x64/LB_API2.dll is missing.")
    print("Please copy LB_API2.dll to this directory and retry")
    quit()
if not os.path.isfile("./DLLs/new/x64/LBUSBDRVD.dll"):
    print("./DLLs/new/x64/LBUSBDRVD.dll is missing.")
    print("Please copy LBUSBDRVD.dll to this directory and retry")
    quit()
if not os.path.isfile("./DLLs/new/x86/LB_API2.dll"):
    print("./DLLs/new/x86/LB_API2.dll is missing.")
    print("Please copy LB_API2.dll to this directory and retry")
    quit()
if not os.path.isfile("./DLLs/new/x86/LBUSBDRVD.dll"):
    print("./DLLs/new/x86/LBUSBDRVD.dll is missing.")
    print("Please copy LBUSBDRVD.dll to this directory and retry")
    quit()
    
if not os.path.isfile("./DLLs/old/x64/LB_API2.dll"):
    print("./DLLs/old/x64/LB_API2.dll is missing.")
    print("Please copy LB_API2.dll to this directory and retry")
    quit()
if not os.path.isfile("./DLLs/old/x86/LB_API2.dll"):
    print("./DLLs/old/x86/LB_API2.dll is missing.")
    print("Please copy LB_API2.dll to this directory and retry")
    quit()



# List of supported sensors
MdlList     = ["NotSet","LB478A","LB479A","LB480A","LB559A","LB579A","LB589A","LB679A","LB680A"]
count       = 0
lb_api2_dll = None
driver      = "none"
bits        = 0

configure_windows()

if (ctypes.sizeof(ctypes.c_voidp) == 8):# x64 else x32
    os.chdir(os.getcwd() + "\\DLLs\\new\\x64\\")
    bits = 64
else:
    os.chdir(os.getcwd() + "\\DLLs\\new\\x86\\")
    bits = 32
    

print("Trying new driver")
try:
    LBUSBDRVD_dll = cdll.LoadLibrary(os.getcwd() + "\\LBUSBDRVD.dll")        
    lb_api2_dll = cdll.LoadLibrary(os.getcwd() + "\\LB_API2.dll")
    print("Loaded new driver")
    driver = "new"
except:
    print("Can't load new driver")

if lb_api2_dll is not None:
    # Get count of attached sensors with new driver
    count = lb_api2_dll.LB_SensorCnt(None)

print ("count = {0}".format( count))

# Load old driver if none found yet
if count == 0:
    lb_api2_dll = None
    if bits == 64:
        os.chdir("..\\..")
        os.chdir(os.getcwd() + "\\old\\x64\\")
    else:
        os.chdir("..\\..")
        os.chdir(os.getcwd() + "\\old\\x86\\")
        
    
    print("Trying old driver")
    try:
        lb_api2_dll = cdll.LoadLibrary(os.getcwd() + "\\LB_API2.dll")
        print("Loaded old driver")
        driver = "old"
    except:
        print("Can't load old driver")

    if lb_api2_dll is not None:
        # Get count of attached sensors with old driver
        count = lb_api2_dll.LB_SensorCnt(None)

# Bail if no sensors are connected to USB
if count == 0:
    print("There are no sensors connected")
    time.sleep(1)
    input("\n\nHit return to quit")
    quit()
else:
    print("There is/are {0} sensor(s) connected\n".format(count))
#========================================================================

print("Using {0} driver".format(driver))

# Create sensor variables
sensor_idx  = ctypes.c_long(0)
sensor_addr = ctypes.c_long(999)
sensor_sNum = create_string_buffer(b"6digit")
sensor_mdl  = ctypes.c_long(0)

# Show address of sensor at sensor_idx
print("List of attached sensors")
print("INDEX\tADDRESS\tSERIAL\tMODEL")

for sensor_idx in range(1,count + 1):
    # Get current address and serial number of sensor at sensor_idx
    sensor_addr = lb_api2_dll.LB_GetAddress_Idx(sensor_idx)
    lb_api2_dll.LB_GetSerNo_Idx(sensor_idx, sensor_sNum)
    lb_api2_dll.LB_GetModelNumber_Idx(sensor_idx, byref(sensor_mdl))
    lb_api2_dll.LB_BlinkLED_Idx(sensor_idx)

    print("{0}\t{1}\t{2}\t{3}".format(sensor_idx, sensor_addr, str(sensor_sNum.value,"utf-8"), MdlList[sensor_mdl.value]))


# Pick sensor from list
sensor_idx = int(input("\n\nPick Index of sensor to read: "))
sensor_addr = lb_api2_dll.LB_GetAddress_Idx(sensor_idx)
lb_api2_dll.LB_GetSerNo_Idx(sensor_idx, sensor_sNum)
lb_api2_dll.LB_BlinkLED_Idx(sensor_idx)
print("Reading sensor at index {0},address {1},serial# {2}\n\n".format(sensor_idx, sensor_addr, str(sensor_sNum.value,"utf-8")))
time.sleep(1)


# Create variables
cw_meas     = ctypes.c_double(-999.0)
currentFreq = ctypes.c_double(0)
newFreq     = ctypes.c_double(2400000000)
currentAver = ctypes.c_long(0)
newAver     = ctypes.c_long(1000)

# Take "bad" measurement since sensor has not yet been initialized
print("Measuring Power before initialization")
err = -999
err = lb_api2_dll.LB_MeasureCW(sensor_addr, ctypes.byref(cw_meas))
print("Power = {0} dBm with error value = {1}\n".format(cw_meas.value, err))

# Initialize sensor
print("Attempting to initialize sensor")
err = -999
err = lb_api2_dll.LB_InitializeSensor_Addr(sensor_addr)
print("Initialized sensor with error value = {0}\n".format(err))

# Get current frequency
err = -999
err = lb_api2_dll.LB_GetFrequency(sensor_addr, ctypes.byref(currentFreq))
print("Showing current Frequency = {0} with error value = {1}\n".format(currentFreq.value, err))

# Set frequency to 2.4GHz
err = -999
err = lb_api2_dll.LB_SetFrequency(sensor_addr, newFreq)
print("Setting Frequency to 2.4GHz with error value = {0}\n".format(err))

# Get current frequency
err = -999
err = lb_api2_dll.LB_GetFrequency(sensor_addr, ctypes.byref(currentFreq))
print("Showing current Frequency = {0} with error value = {1}\n".format(currentFreq.value, err))


# Get current number of averages
err = -999
err = lb_api2_dll.LB_GetAverages(sensor_addr, ctypes.byref(currentAver))
print("Showing current averages = {0} with error value = {1}\n".format(currentAver.value, err))

# Set averages to 1000
err = -999
err = lb_api2_dll.LB_SetAverages(sensor_addr, newAver)
print("Setting averages to 1000 with error value = {0}\n".format(err))

# Get current averages
err = -999
err = lb_api2_dll.LB_GetAverages(sensor_addr, ctypes.byref(currentAver))
print("Showing current averages = {0} with error value = {1}\n".format(currentAver.value, err))


# Take "good" measurement
print("Measuring Power after initialization")
err = -999
err = lb_api2_dll.LB_MeasureCW(sensor_addr, ctypes.byref(cw_meas))
print("Power = {0} dBm with error value = {1}\n".format(cw_meas.value, err))


input("\n\nHit return to quit")
quit()

