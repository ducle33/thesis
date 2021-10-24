#!/usr/bin/env python3


# ## BASE_CONTROL 
# Author: Linh Nguyen (dlinhvn@gmail.com)
# Created: 3/10/2021
# Description: This file is used for communication stack between ROS and STM32. 
# Its role is to get cmd_vel and publishing tf msgs 
# ###

import threading
import serial
from math import cos, sin
import struct
import signal
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from rospy import Time 
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

connected = False
#exit_event = threading.Event()
port = '/dev/ttyUSB0'
baud = 115200

#serial_port = serial.Serial(port, baud, timeout=0)

vel_x = 0
vel_z = 0
x = 0
y = 0
theta = 0
br = tf.TransformBroadcaster()
tf_ready = 0

def parse_tx(vel_x, vel_z):
    # convert to byte arrays
    vel_x_ba = bytearray(struct.pack(">f", vel_x))  
    vel_z_ba = bytearray(struct.pack(">f", vel_z))  

    dframe = vel_x_ba + vel_z_ba

    checksum = 0

    for mem in dframe:
        checksum += mem

    dframe += bytearray(struct.pack(">i", checksum)) 
    dframe = bytearray([1, 2]) + dframe+ bytearray([0x0D, 0x0A]) #CR LF

    print([ "0x%02x" % b for b in dframe ])

    return dframe


def tf_publish():
    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, 0),
                 tf.transformations.quaternion_from_euler(0, 0, theta),
                 rospy.Time.now(),
                 "base_link",
                 "odom")


def handle_data(data):
    global x, y, theta, tf_ready
    # Get checksum and resolve frame 
    checksum = sum(data[2:26])
    # test = struct.unpack('>i', data[26:28])[0]
    if (checksum == struct.unpack('>i', data[26:30])[0]): #
        # Unpack with big endian
        vx = struct.unpack('>d', data[2:10])[0]
        vth = struct.unpack('>d', data[10:18])[0]
        dt = struct.unpack('>d', data[18:26])[0]

        x += 0
        y += 0
        theta += vth * dt
        tf_ready = 1
        
        # print(f"TF update : x = {x}, y = {y}, theta = {theta}")
        # print(vx, vth, dt, checksum)
        if tf_ready:
            br.sendTransform((x, y, 0),
                 tf.transformations.quaternion_from_euler(0, 0, theta),
                 rospy.Time.now(),
                 "base_link",
                 "odom")



def read_from_port(ser):
    global connected, exit_event
    temp_data = 0
    while not connected:
        #serin = ser.read()
        connected = True
        while True:
            reading = ser.readline()
            dlen = len(reading)
            if (dlen>0):
                if (reading[0]==1 and reading[1]==2):
                    if dlen<32 and temp_data==0:
                        temp_data = reading
                    elif dlen==32:
                        handle_data(reading)
                        
                else:
                    if dlen<32 and temp_data!=0 and len(temp_data+reading)==32:
                        handle_data(temp_data+reading)
                        temp_data = 0
                    else:
                        pass

            time.sleep(0.02)
            if exit_event.is_set():
                break


#### Ultilities for ros node
def cmd_vel_callback(msg):
    data = parse_tx(msg.linear.x, msg.angular.z)
    sync = bytearray([0x16, 0x16, 0x16])
    #serial_port.write(sync)
    #serial_port.write(data)
    
def ros_worker():
    global br

    rospy.init_node('base_control_listener', anonymous=True)
    print("Init node success ...")
    cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    print("Subscribe /cmd_vel success ...")
    tf_ready = 1
    print("TF publishing ready ...")
    x = 0
    y = 0
    theta = 0.5
    vx = 0
    vy = 0
    vth = 0
    while True:
        current_time = rospy.Time.now()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)

        br.sendTransform((x, y, 0), odom_quat, current_time,"base_link","odom")
        
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


        time.sleep(0.1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

### Threading ultilities
def signal_handler(signum, frame):
    exit_event.set()


if __name__ == '__main__':

    #signal.signal(signal.SIGINT, signal_handler)
    #thread = threading.Thread(target=read_from_port, args=(serial_port,))
    #thread.start()
    
    try:
        ros_worker()
    except KeyboardInterrupt:
        #thread.join()
        sys.exit(0)
        


