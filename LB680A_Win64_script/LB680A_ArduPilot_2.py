import ctypes
import time
import os
import sys
import platform
from ctypes import *
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect
import glob
import serial

##############################################################
###################### SUBFUNCTIONS ##########################

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

def search_serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def mavlinkIsConnected():
    ports = search_serial_ports()
    try:
        # connect to vehicle
        global vehicle 
        for port in ports:
            # Connect to the available port
            vehicle = utility.mavlink_connection(device=port, baud=115200, source_system=1, source_component=191)
            # wait for a heartbeat
            if vehicle.wait_heartbeat(timeout=5) != None:
                print(port)
                return True
        print("ERR: No mavlink found on available ports")
        return False
    except:
        print("ERR: An exception occurred while searching for available ports")
        return False
        
        
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

# Establish Mavlink connection (No need to hard-code the port label)
while mavlinkIsConnected() == False:
    time.sleep(5)

# inform user on Python Terminal
print("Connected to system:", vehicle.target_system, ", component:", vehicle.target_component)

# inform user on GCS
text = f"ARRC computer connected"
TXT_message = dialect.MAVLink_statustext_message(severity=dialect.MAV_SEVERITY_INFO,
                                                 text=text.encode("utf-8"))
vehicle.mav.send(TXT_message)

##########################################################################
###################### LIST OF REQUEST MESSAGES ##########################

# create SENSOR PARAMS request message command
request_SNSRFREQ_message_command = dialect.MAVLink_param_request_read_message(
                                                                target_system=vehicle.target_system,
                                                                target_component=vehicle.target_component,
                                                                param_id="USR_LB_DFREQ".encode("utf-8"),
                                                                param_index=-1)

request_SNSRAVG_message_command = dialect.MAVLink_param_request_read_message(
                                                                target_system=vehicle.target_system,
                                                                target_component=vehicle.target_component,
                                                                param_id="USR_LB_DAVG".encode("utf-8"),
                                                                param_index=-1)

request_SNSRMODE_message_command = dialect.MAVLink_param_request_read_message(
                                                                target_system=vehicle.target_system,
                                                                target_component=vehicle.target_component,
                                                                param_id="USR_LB_DMODE".encode("utf-8"),
                                                                param_index=-1)

vehicle.mav.send(request_SNSRFREQ_message_command)
SNSRFREQ_message = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname,
                                 blocking=True)

vehicle.mav.send(request_SNSRAVG_message_command)
SNSRAVG_message = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname,
                                 blocking=True)

vehicle.mav.send(request_SNSRMODE_message_command)
SNSRMODE_message = vehicle.recv_match(type=dialect.MAVLink_param_value_message.msgname,
                                 blocking=True)

SNSRFREQ_message = SNSRFREQ_message.to_dict()
SNSRAVG_message = SNSRAVG_message.to_dict()
SNSRMODE_message = SNSRMODE_message.to_dict()

DFREQ = int(SNSRFREQ_message["param_value"])
DAVG = int(SNSRAVG_message["param_value"])
DMODE = int(SNSRMODE_message["param_value"])

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
print("Setting Frequency to", DFREQ, "MHz with error value = {0}\n".format(err))

# Set desired average length
err = -999
err = lb_api2_dll.LB_SetAverages(sensor_addr, newAver)
print("Setting averages to", DAVG,  "with error value = {0}\n".format(err))



#========================================================================
#               LB measurements and Data streaming
#========================================================================
#last_meas = time.time()
last_msg = time.time()

while (True):

    if(Pulse_NotCW == False):
        # Take CW measurement
        lb_api2_dll.LB_MeasureCW(sensor_addr, ctypes.byref(cw_meas))
        #print("Power = {0} dBm\n".format(cw_meas.value))
    else:
        # Take Pulse measurement
        lb_api2_dll.LB_MeasurePulse(sensor_addr, ctypes.byref(pulse_pwr), ctypes.byref(pulse_pkpwr), ctypes.byref(pulse_avgpwr), ctypes.byref(pulse_dutycycle))
        # print("Pulse Power = {0} dBm\n".format(pulse_pwr.value))
        # print("Peak Power = {0} dBm\n".format(pulse_pkpwr.value))
        # print("Average Power = {0} dBm\n".format(pulse_avgpwr.value))
        # print("Duty Cycle = {0}\n".format(pulse_dutycycle.value))

    # Send Mavlink messege to Pixhawk
    if(time.time() - last_msg > 0.01 and vehicle != None):
        # Pack ARRC's message and send it
        if(Pulse_NotCW == True):
            ARRC_data = dialect.MAVLink_arrc_sensor_raw_message(10,0,0,0,pulse_pwr.value,pulse_pkpwr.value,pulse_avgpwr.value,pulse_dutycycle.value)
            vehicle.mav.send(ARRC_data)
        else:
            ARRC_data = dialect.MAVLink_arrc_sensor_raw_message(10,0,0,0,cw_meas.value,0,0,0)
            vehicle.mav.send(ARRC_data)
        last_msg = time.time()

    #print("Loop time = {0} sec".format(time.time() - last_meas))
    #last_meas = time.time()