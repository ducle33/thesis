#!/usr/bin/env python3
# ## BASE_CONTROL 
# Author: Linh Nguyen (dlinhvn@gmail.com)
# Created: 3/10/2021
# Description: This file is used for communication stack between ROS and STM32. 
# Its role is to get cmd_vel and publishing tf msgs.
# Bytes order: Big Endian 
# ###

import threading
import serial
import math
import struct
import signal
import sys
import time
import tf
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rospy import Time 
from RPi import  GPIO


connected = False
exit_event = threading.Event()
port = '/dev/ttyS0'
baud = 115200
serial_port = serial.Serial(port, baud, timeout=0)

ENABLE_ODOM = True
STM_RESET_PIN = 4   # High-on-reset GPIO pin

vel_x = 0
vel_z = 0
x = 0
y = 0
theta = 0
vx = 0
vy = 0
dt = 0
tf_ready = 0
br = tf.TransformBroadcaster()
odom_pub = 0

# GPIO Setup
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(STM_RESET_PIN, GPIO.OUT)


def parse_tx(vel_x, vel_z):
    print(f"[BASE CONTROL]New cmd_vel msgs -> x = {vel_x} ; z = {vel_z}")
    # convert to byte arrays, ">" stand for big-edian float
    vel_x_ba = bytearray(struct.pack(">f", vel_x))  
    vel_z_ba = bytearray(struct.pack(">f", vel_z))  

    dframe = vel_x_ba + vel_z_ba

    checksum = 0

    for mem in dframe:
        checksum += mem

    dframe += bytearray(struct.pack(">i", checksum)) 
    dframe = bytearray([1, 2]) + dframe+ bytearray([0x0D, 0x0A]) #CR LF

    return dframe


def tf_publish():
    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, 0),
                 tf.transformations.quaternion_from_euler(0, 0, theta),
                 rospy.Time.now(),
                 "base_link",
                 "odom")


def handle_data(data):
    global x, y, theta, vx, vy, dt, tf_ready, odom_pub
    # Get checksum and resolve frame 
    checksum = sum(data[2:26])
    # test = struct.unpack('>i', data[26:28])[0]
    if (checksum == struct.unpack('>i', data[26:30])[0]) and tf_ready: #
            # Unpack with big endian
            left_speed = struct.unpack('>d', data[2:10])[0]
            right_speed = struct.unpack('>d', data[10:18])[0]
            dt = struct.unpack('>d', data[18:26])[0]

            vx = (right_speed + left_speed) * ( math.pi * 0.065 ) / (2 * 60)
            vth = (right_speed - left_speed) * ( math.pi * 0.065 ) / (0.205 * 60)
            dt = 20/1000.0

            x += math.cos(theta)*vx*dt
            y += math.sin(theta)*vx*dt
            theta += vth * dt
            
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

            current_time = rospy.Time.now()
            
            # print(f"[BASE CONTROL] TF update : x = {x}, y = {y}, theta = {theta}")
            #br.sendTransform((x, y, 0), odom_quat, current_time,"base_link","map")
            
            
            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            odom_pub.publish(odom)
            


def read_from_port(ser):
    global connected, exit_event
    temp_data = []
    next_frame = False
    last_reading = ''
    prev_reading = ''

    while not connected:
        #serin = ser.read()
        connected = True
        while True:
            reading = ser.read()
#            print(type(reading))
#            time.sleep(0.1)
            if (len(reading) > 0):
                if (reading == b'\x02') and (last_reading == b'\x01') and (next_frame ==False):
                    temp_data.append(last_reading)
                    next_frame = True
                if next_frame == True :
                    temp_data.append(reading)
                    if (len(temp_data) == 32) and (last_reading == b'\r') and (reading == b'\n'):
                        # Data valid 
                        handle_data(b''.join(temp_data))
                        next_frame = False
                        temp_data = []
                        pass
                    elif len(temp_data)> 32:
                        next_frame = False
                        temp_data = []
                    else:
                        pass      
                else:
                    temp_data = []
                prev_reading = last_reading
                last_reading = reading

            if exit_event.is_set():
                break

#### Ultilities for ros node
def cmd_vel_callback(msg):
    data = parse_tx(msg.linear.x, msg.angular.z)
    sync = bytearray([0x16, 0x16, 0x16])
    serial_port.write(sync)
    serial_port.write(data)
    
def ros_worker():
    global br, tf_ready, ENABLE_ODOM, STM_RESET_PIN, odom_pub

    rospy.init_node('base_control_listener', anonymous=True)
    print("[BASE CONTROL] -> Init node success ...")
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    print("[BASE CONTROL] -> Subscribe /cmd_vel success. Reset STM32 ...")
#    GPIO.output(STM_RESET_PIN, GPIO.LOW)
#    GPIO.output(STM_RESET_PIN, GPIO.HIGH)
    if (ENABLE_ODOM):
        odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        tf_ready = 1
        start = bytearray([0x16, 0x2, 0x16])
        serial_port.write(start)
        print("[BASE CONTROL] -> TF publishing ready ...")
    else:
        print("[BASE CONTROL][ WARN] Real Odom DISABLED, using VISUAL ODOM")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

### Threading ultilities
def signal_handler(signum, frame):
    exit_event.set()


if __name__ == '__main__':
    print(f"[BASE CONTROL] -> Serial PORT open at \n\t{port} ")
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=read_from_port, args=(serial_port,))
    thread.start()
    
    # Check ROS MASTER available ?
    ros_master_available = False
    while not rospy.is_shutdown() and not ros_master_available:
        try:  
            # query ROS Master for published topics   
            rospy.get_published_topics()  
            ros_master_available = True
        except:        
            print("[BASE CONTROL] Waiting for roscore ...")  
        time.sleep(0.5)  # sleep for 0.1 seconds   
    
    try:
        ros_worker()
    except KeyboardInterrupt:
        thread.join()
        #GPIO.cleanup()
        sys.exit()
        


