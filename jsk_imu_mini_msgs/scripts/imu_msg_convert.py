#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from jsk_imu_mini_msgs.msg import Imu as Imu_jsk
from scipy.spatial.transform import Rotation as R

"""
convert jsk_imu_mini_msgs/Imu messages to sensor_msgs/Imu and sensor_msgs/MagneticField messages

subscribes: /imu
publishes: /imu/data_raw, /imu/mag
"""

def array_to_Vector3(array, vector3):
    [vector3.x, vector3.y, vector3.z] = array

def euler_to_quaternion(euler, quaternion):
    # convert roll pitch yaw euler angles to quaternion
    q = R.from_euler('xyz', euler).as_quat()
    quaternion.x = q[0]
    quaternion.y = q[1]
    quaternion.z = q[2]
    quaternion.w = q[3]


imu_msg = Imu()
mag_msg = MagneticField()
imu_msg.header.frame_id = "world"
mag_msg.header.frame_id = "world"
def jsk_imu_callback(jsk_imu_msg):
    imu_msg.header.stamp = jsk_imu_msg.stamp
    mag_msg.header.stamp = jsk_imu_msg.stamp
    array_to_Vector3(jsk_imu_msg.gyro_data, imu_msg.angular_velocity)
    array_to_Vector3(jsk_imu_msg.acc_data, imu_msg.linear_acceleration)
    array_to_Vector3(jsk_imu_msg.mag_data, mag_msg.magnetic_field)
    euler_to_quaternion(jsk_imu_msg.angles, imu_msg.orientation)
    imu_pub.publish(imu_msg)
    mag_pub.publish(mag_msg)



rospy.init_node('imu_msg_convert')
imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=1)
mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=1)
rospy.Subscriber('/imu', Imu_jsk, jsk_imu_callback)

rospy.spin()