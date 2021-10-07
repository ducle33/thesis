# ## BASE_CONTROL 
# Author: Linh Nguyen (dlinhvn@gmail.com)
# Created: 3/10/2021
# Description: This file is used for communication stack between ROS and STM32. 
# Its role is to get cmd_vel and publishing tf msgs 
# ###

import threading
import serial
import struct
import signal
import sys
import time
import rospy
from geometry_msgs.msg import Twist
import tf

connected = False
exit_event = threading.Event()
port = '/dev/ttyUSB1'
baud = 115200

serial_port = serial.Serial(port, baud, timeout=0)

vel_x = 0.5
vel_z = 0.1
x = 0
y = 0
theta = 0

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


def handle_data(data):
    global x, y, theta
    # Get checksum and resolve frame 
    checksum = sum(data[2:26])
    # test = struct.unpack('>i', data[26:28])[0]
    if (checksum == struct.unpack('>i', data[26:30])[0]): #
        # Unpack with big endian
        vx = struct.unpack('>d', data[2:10])[0]
        vth = struct.unpack('>d', data[10:18])[0]
        dt = struct.unpack('>d', data[18:26])[0]

        x += cos(theta)*vx*dt;
        y += sin(theta)*vx*dt;
        theta += vth * dt;
        # print(vx, vth, dt, checksum)

        br = tf.TransformBroadcaster()
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
                    if dlen<32 and temp_data!=0:
                        handle_data(temp_data+reading)
                        temp_data = 0

            time.sleep(0.02)
            if exit_event.is_set():
                break


#### Ultilities for ros node
def cmd_vel_callback(msg):
    data = parse_tx(msg.linear.x, msg.angular.z)
    sync = bytearray([0x16, 0x16, 0x16])
    serial_port.write(sync)
    serial_port.write(data)
    
def ros_worker():
    rospy.init_node('linhsbot_tf_broadcaster')
    rospy.init_node('base_control_listener', anonymous=True)
    print("Init node success ...")
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
    print("Subscribe /cmd_vel success ...")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

### Threading ultilities
def signal_handler(signum, frame):
    exit_event.set()


if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=read_from_port, args=(serial_port,))
    thread.start()
    try:
        ros_worker()
    except KeyboardInterrupt:
        thread.join()
        sys.exit()
        


