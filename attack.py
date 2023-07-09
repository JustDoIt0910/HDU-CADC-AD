from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative,Command
from math import radians, cos, sin, asin, sqrt
import RPi.GPIO as GPIO
import serial
import time


P_SERVO = 4
P_SERVO2 = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(P_SERVO, GPIO.OUT)
GPIO.setup(P_SERVO2, GPIO.OUT)
GPIO.output(P_SERVO2, GPIO.HIGH)
GPIO.output(P_SERVO, GPIO.HIGH)



connect_string = '/dev/ttyUSB0'
vehicle = connect(connect_string, wait_ready=True, baud=57600)
ser = serial.Serial("/dev/ttyAMA0", 9600)
ser.flushInput()
ser.flushOutput()


def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist
    
def upload_mission(aFileName):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmission(aFileName)
    
    print("\nUpload mission from a file: %s" % aFileName)
    #Clear existing mission from vehicle
    print('Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print('Upload mission')
    vehicle.commands.upload()
    vehicle.mode = VehicleMode("RTL")
    time.sleep(2)
    vehicle.mode = VehicleMode("AUTO")


def tp_distance(lat11, lon11, lat22, lon22):
    lon11, lat11, lon22, lat22 = map(
        radians, [float(lon11), float(lat11), float(lon22), float(lat22)])
    dlon = lon22-lon11
    dlat = lat22-lat11
    a = sin(dlat/2)**2 + cos(lat11) * cos(lat22) * sin(dlon/2)**2
    distance = 2*asin(sqrt(a))*6378137
    distance = round(distance, 3)
    return distance

# TODO 修改投水舵机控制
def unload():
    GPIO.output(P_SERVO, GPIO.HIGH)
    GPIO.output(P_SERVO2, GPIO.LOW)

def load():
    GPIO.output(P_SERVO2, GPIO.HIGH)
    GPIO.output(P_SERVO, GPIO.LOW)


def drop(tar_lat, tar_lon):
    height = 0
    while True:
        V = vehicle.groundspeed
        new_height = vehicle.location.global_relative_frame.alt
        if new_height > 0:
            height = new_height
        # print("height:", height)
        hstr = "height: {}".format(height)
        # ser.write(hstr.encode())
        # ser.flush()

        # print("V: ", V)
        vstr = "v: {}".format(V)
        # ser.write(vstr.encode())
        # ser.flush()

        x = V*sqrt(2*height/9.8)
        dis = tp_distance(tar_lat, tar_lon, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
        # print("to tar:%s"% x)
        # print("now:%s"% )
        totar = "to tar: {}".format(x)
        now = "now :{}\n".format(dis)
        # ser.write(totar.encode())
        # ser.flush()
        # ser.write(now.encode())
        # ser.flush()
        if (dis <= x + 3 and height <= 25):
            GPIO.output(P_SERVO, GPIO.HIGH)
            GPIO.output(P_SERVO2, GPIO.LOW)
            ser.write("bomb away".encode())
            ser.flush()
            break
        time.sleep(0.1)


def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist
    

if __name__ == '__main__':

    # 目标GPS坐标
    tar_lat1 = 30.3058363
    tar_lon1 = 120.4201099

    tar_lat2 = 0.0
    tar_lon2 = 0.0

    
    tar_lat = 0.0
    tar_lon = 0.0


    ser.write("waiting for command...\n".encode())
    ser.flush()
    while True:
        recv_num = ser.inWaiting()
        if recv_num > 7:
            recv_buffer = ser.read(recv_num)
            recv_data = recv_buffer.decode()[:-1]
            ser.flushInput()
            if recv_data[:6] != "switch":
                continue

            if recv_data[6] == "1":
                tar_lat = tar_lat1
                tar_lon = tar_lon1
                ser.write("switching to attacking route 1\n".encode())
                ser.flush()
                upload_mission("wp1.txt")

            elif recv_data[6] == "2":
                tar_lat = tar_lat2
                tar_lon = tar_lon2
                ser.write("switching to attacking route 2\n".encode())
                ser.flush()
                upload_mission("wp2.txt")
            drop(tar_lat, tar_lon)
        
        elif recv_num > 4:
            recv_buffer = ser.read(recv_num)
            recv_data = recv_buffer.decode()[:-1]
            if recv_data == "load":
                load()
            elif recv_data == "unload":
                unload()
        time.sleep(0.2)
