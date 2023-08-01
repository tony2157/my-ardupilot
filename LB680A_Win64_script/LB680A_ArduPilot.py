import ctypes
import time
import os
import sys
import platform
from ctypes import *
from pymavlink import mavutil

def configure_windows():
    """
        Set paths to DLLs
    """
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
#                         Look for LB sensor
#========================================================================
print("Using {0} driver".format(driver))

# Create sensor variables
sensor_idx  = ctypes.c_long(0)
sensor_addr = ctypes.c_long(999)
sensor_sNum = create_string_buffer(b"6digit")
sensor_mdl  = ctypes.c_long(0)

for sensor_idx in range(1,count + 1):
    # Get current address and serial number of sensor at sensor_idx
    sensor_addr = lb_api2_dll.LB_GetAddress_Idx(sensor_idx)
    lb_api2_dll.LB_GetSerNo_Idx(sensor_idx, sensor_sNum)
    lb_api2_dll.LB_GetModelNumber_Idx(sensor_idx, byref(sensor_mdl))
    lb_api2_dll.LB_BlinkLED_Idx(sensor_idx)

    print("{0}\t{1}\t{2}\t{3}".format(sensor_idx, sensor_addr, str(sensor_sNum.value,"utf-8"), MdlList[sensor_mdl.value]))


# Pick sensor from list
sensor_idx = int(1)  # Always choose first sensor. BEFORE: int(input("\n\nPick Index of sensor to read: "))
sensor_addr = lb_api2_dll.LB_GetAddress_Idx(sensor_idx)
lb_api2_dll.LB_GetSerNo_Idx(sensor_idx, sensor_sNum)
lb_api2_dll.LB_BlinkLED_Idx(sensor_idx)
print("Reading sensor at index {0},address {1},serial# {2}\n\n".format(sensor_idx, sensor_addr, str(sensor_sNum.value,"utf-8")))
time.sleep(1)


#========================================================================
#               Init Mavlink and look for autopilot
#========================================================================

# Start connection with Pixhawk through Mavlink
try:
    ARRC_mav_connection = mavutil.mavlink_connection('com5', baud=115200, source_system=1, source_component=191, dialect='ARRC_Mavlink')
    #ARRC_mav_connection = mavutil.mavserial('COM12', baud=115200, source_system=1, source_component=191)
except:
    print("No Mavlink connection achieved")
    ARRC_mav_connection = None

if (ARRC_mav_connection != None):
    # Wait for hearbeat from Pixhawk
    tictoc = time.time()
    PX4_beat = ARRC_mav_connection.wait_heartbeat(timeout=1000)

    print("Mavlink connection: "+ str(PX4_beat))
    print("Heartbeat system: sysID %u compID %u" % (ARRC_mav_connection.target_system, ARRC_mav_connection.target_component))

    # Send heartbeat to confirm handshake
    ARRC_mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    # Wait for config message 
    msg = ARRC_mav_connection.recv_match(type='ARRC_SENSOR_RAW', timeout=5000)
    if not msg:
        DFREQ = 3070
        DAVG = 20
        DMODE = 1
    elif msg.get_type() == "BAD_DATA":
        DFREQ = 3070
        DAVG = 20
        DMODE = 1
    else:
        DFREQ = msg.dfreq
        DAVG = msg.davg
        DMODE = msg.dmode
else:
    # Default values
    DFREQ = 3070
    DAVG = 20
    DMODE = 1


#========================================================================
#                          Init LB sensor
#========================================================================
# Create variables
cw_meas         = ctypes.c_double(-999.0)
pulse_pwr       = ctypes.c_double(-999.0)
pulse_pkpwr     = ctypes.c_double(-999.0)
pulse_avgpwr    = ctypes.c_double(-999.0)
pulse_dutycycle = ctypes.c_double(0)
newFreq         = ctypes.c_double(DFREQ*1000000)
newAver         = ctypes.c_long(DAVG)
if(DMODE > 0.5):
    Pulse_NotCW = True
else:
    Pulse_NotCW = False

# Take "bad" measurement since sensor has not yet been initialized
print("Measuring Power before initialization")
err = -999
err = lb_api2_dll.LB_MeasureCW(sensor_addr, ctypes.byref(cw_meas))
print("Power = {0} dBm with error value = {1}\n".format(cw_meas.value, err))

# Initialize LB sensor
print("Attempting to initialize sensor")
err = -999
err = lb_api2_dll.LB_InitializeSensor_Addr(sensor_addr)
print("Initialized sensor with error value = {0}\n".format(err))

# Set desired frequency
err = -999
err = lb_api2_dll.LB_SetFrequency(sensor_addr, newFreq)
print("Setting Frequency to {0}MHz with error value = {1}\n".format(newFreq.value/1000000, err))

# Set desired average length
err = -999
err = lb_api2_dll.LB_SetAverages(sensor_addr, newAver)
print("Setting averages to {0} with error value = {1}\n".format(newAver.value, err))



#========================================================================
#               LB measurements and Data streaming
#========================================================================
last_meas = time.time()
last_msg = time.time()
last_beat = time.time()

while (True):

    if(Pulse_NotCW == False):
        # Take CW measurement
        lb_api2_dll.LB_MeasureCW(sensor_addr, ctypes.byref(cw_meas))
        #print("Power = {0} dBm\n".format(cw_meas.value))
    else:
        # Take Pulse measurement
        lb_api2_dll.LB_MeasurePulse(sensor_addr, ctypes.byref(pulse_pwr), ctypes.byref(pulse_pkpwr), ctypes.byref(pulse_avgpwr), ctypes.byref(pulse_dutycycle))
        print("Pulse Power = {0} dBm\n".format(pulse_pwr.value))
        print("Peak Power = {0} dBm\n".format(pulse_pkpwr.value))
        print("Average Power = {0} dBm\n".format(pulse_avgpwr.value))
        print("Duty Cycle = {0}\n".format(pulse_dutycycle.value))

    # Send Mavlink messege to Pixhawk
    if(time.time() - last_msg > 0.016 and ARRC_mav_connection != None):
        # Pack ARRC's message and send it
        if(Pulse_NotCW == True):
            ARRC_mav_connection.mav.arrc_sensor_raw_send(10,0,0,0,pulse_pwr.value,pulse_pkpwr.value,pulse_avgpwr.value,pulse_dutycycle.value)
        else:
            ARRC_mav_connection.mav.arrc_sensor_raw_send(10,0,0,0,cw_meas.value,0,0,0)
        last_msg = time.time()

    # Send Heartbeat to Pixhawk every second
    if(time.time() - last_beat > 0.95 and ARRC_mav_connection != None):
        ARRC_mav_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        last_beat = time.time()

    print("Loop time = {0} sec".format(time.time() - last_meas))
    last_meas = time.time()