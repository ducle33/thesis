#!/usr/bin/env python3
import sys
import time
import smbus
import struct
import rospy
import numpy as np
import math
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_about_axis
from mpu6050_addr import *
from tf.transformations import quaternion_from_euler

# Can be set from launch file
ADDR = None
bus = None
IMU_FRAME = None
dtTimer = None 
temp_pub = None
imu_pub = None

gyroRoll = gyroPitch = gyroYaw = 0
roll = pitch = yaw = 0
#offset 
ax = int(-705)
ay = int(120)
az = int(1078)
gx = int(89)
gy = int(38)
gz = int(-11)
# read_word and read_word_2c from http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html


def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr+1)
    val = (high << 8) + low
    return val
        
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val


def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = read_word_2c(TEMP_H)/340.0 + 36.53
    temp_msg.header.stamp = rospy.Time.now()
    temp_pub.publish(temp_msg)


def publish_imu(timer_event):
    global dtTimer

    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    # Read the acceleration vals
    accel_x = (read_word_2c(ACCEL_XOUT_H))/ 16384.0
    accel_y = (read_word_2c(ACCEL_YOUT_H))/ 16384.0
    accel_z = (read_word_2c(ACCEL_ZOUT_H))/ 16384.0

    # Read the gyro vals
    gyro_x = (read_word_2c(GYRO_XOUT_H))/ 131.0
    gyro_y = (read_word_2c(GYRO_YOUT_H))/ 131.0
    gyro_z = (read_word_2c(GYRO_ZOUT_H))/ 131.0
    
    # Get delta time and record time for next call
    dt = time.time() - dtTimer
    dtTimer = time.time()
    roll, pitch, yaw = compFilter(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt, tau = 0.8)
    q = quaternion_from_euler(roll, pitch, yaw)
    imu_msg.orientation.x = q[0]
    imu_msg.orientation.y = q[1]
    imu_msg.orientation.z = q[2]
    imu_msg.orientation.w = q[3]
 
    # Load up the IMU message
    '''o = imu_msg.orientation
    o.x, o.y, o.z, o.w = orientation'''

    imu_msg.linear_acceleration.x = accel_x*9.80665 
    imu_msg.linear_acceleration.y = accel_y*9.80665 
    imu_msg.linear_acceleration.z = accel_z*9.80665 

    imu_msg.angular_velocity.x = gyro_x*math.pi/180
    imu_msg.angular_velocity.y = gyro_y*math.pi/180
    imu_msg.angular_velocity.z = gyro_z*math.pi/180

    imu_msg.header.stamp = rospy.Time.now()
    imu_pub.publish(imu_msg)
    
    
def setAxOffset(data):
    bus.write_byte_data(ADDR, MPU6050_RA_XA_OFFS_H, (data>>8))
    bus.write_byte_data(ADDR, MPU6050_RA_XA_OFFS_L_TC, (data>>0))
    
def setAyOffset(data): 
    bus.write_byte_data(ADDR, MPU6050_RA_YA_OFFS_H, (data>>8))
    bus.write_byte_data(ADDR, MPU6050_RA_YA_OFFS_L_TC,(data>>0))
    
def setAzOffset(data):
    bus.write_byte_data(ADDR, MPU6050_RA_ZA_OFFS_H, (data>>8))
    bus.write_byte_data(ADDR, MPU6050_RA_ZA_OFFS_L_TC, (data>>0))
    
def setGxOffset(data):
    bus.write_byte_data(ADDR, MPU6050_RA_XG_OFFS_USRH, (data>>8))
    bus.write_byte_data(ADDR, MPU6050_RA_XG_OFFS_USRL, (data>>0))
    
def setGyOffset(data):
    bus.write_byte_data(ADDR, MPU6050_RA_YG_OFFS_USRH, (data>>8))
    bus.write_byte_data(ADDR, MPU6050_RA_YG_OFFS_USRL, (data>>0))
    
def setGzOffset(data):
    bus.write_byte_data(ADDR, MPU6050_RA_ZG_OFFS_USRH, (data>>8))
    bus.write_byte_data(ADDR, MPU6050_RA_ZG_OFFS_USRL, (data>>0))

def compFilter( ax, ay, az, gx, gy, gz, dt, tau):
    global gyroRoll, gyroPitch, gyroYaw, roll, pitch, yaw
    # Acceleration vector angle
    accPitch = math.degrees(math.atan2(ay, math.sqrt(az**2 + ax**2)))
    accRoll = math.degrees(math.atan2(ax, math.sqrt(az**2 + ay**2)))
    # Gyro integration angle
    gyroRoll -= gy * dt
    gyroPitch += gx * dt
    gyroYaw += gz * dt
    yaw = gyroYaw
    # Comp filter
    roll = (tau)*(roll - gy*dt) + (1-tau)*(accRoll)
    pitch = (tau)*(pitch + gx*dt) + (1-tau)*(accPitch)
    # Print data
    #print(" R: " + str(round(roll,1)) \
    #    + " P: " + str(round(pitch,1)) \
    #    + " Y: " + str(round(yaw,1)))
    return roll, pitch, yaw



# Test connection
def MPU_Test_Conn():
    high = bus.read_byte_data(ADDR, MPU6050_RA_WHO_AM_I)
    high = (high>>1) & 0b00111111
    if high == 0x34:
        print("Connection response: OK")
        return 0
    else:
        return 1

def MPU_Init():
    # Set clock source PLL as X Gyro reference - datasheet p.40
    bus.write_byte_data(ADDR, 0x6B, 0x00)
    # Configure the accelerometer
    bus.write_byte_data(ADDR, 0x1C, 0x00)
    # Configure the gyro
    bus.write_byte_data(ADDR, 0x1B, 0x00)
    setAxOffset(0)
    setAyOffset(0)
    setAzOffset(0)
    setGxOffset(0)
    setGyOffset(0)
    setGzOffset(0)
    return 0



if __name__ == '__main__':

    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', MPU6050_ADDRESS_AD0_LOW)
    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    MPU_Init()
    if MPU_Test_Conn():
        print("[IMU NODE ] MPU6050 not detected")
        sys.exit(0)
    print("[IMU NODE ] Running with offset: ", ax, ay, az, gx, gy, gz)
    setAxOffset(ax)
    setAyOffset(ay)
    setAzOffset(az)
    setGxOffset(gx)
    setGyOffset(gy)
    setGzOffset(gz)
    
    dtTimer = time.time()

    temp_pub = rospy.Publisher('temperature', Temperature,queue_size=10)
    imu_pub = rospy.Publisher('imu/data_raw', Imu,queue_size=10)
    imu_timer = rospy.Timer(rospy.Duration(0.1), publish_imu)
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)
    print("[IMU NODE ] Start publishing")
    rospy.spin()
