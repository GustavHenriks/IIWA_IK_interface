#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from robot_kinematic_python.Robot_Model import Kinematic
from robot_kinematic_python.Structure import Pos, M_PI_2, M_PI
import tf.transformations as t
import numpy as np

def to_rad(x):
    return (float(x)/180 * M_PI)

def callback(pos):
    quaternion = (
        pos.orientation.x,
        pos.orientation.y,
        pos.orientation.z,
        pos.orientation.w)
    euler = t.euler_from_quaternion(quaternion)   
    print("Roll :  " + str(euler[0]) + " \nPitch : " + str(euler[1]) + "\nYaw   : " + str(euler[2]))


rospy.init_node("show_yaw", anonymous=True)
rospy.Subscriber("/IIWA/Real_E_Pos", Pose, callback)
rospy.spin()


# Kinematic_Chain = Kinematic(7)

# T0 = np.zeros((4, 4))
# T0[0, 0] = 1
# T0[1, 1] = 1
# T0[2, 2] = 1
# T0[3, 3] = 1
# T0[0, 3] = 0
# T0[1, 3] = 0
# T0[2, 3] = 0
# Kinematic_Chain.setDH(
#     0, 0.0, 0.34, M_PI_2, 0.0, 1, to_rad(-170.), to_rad(170.), to_rad(98.0))
# Kinematic_Chain.setDH(
#     1, 0.0, 0.00, -M_PI_2, 0.0, 1, to_rad(-110.), to_rad(110.), to_rad(98.0))
# Kinematic_Chain.setDH(
#     2, 0.0, 0.40, -M_PI_2, 0.0, 1, to_rad(-170.), to_rad(170.), to_rad(100.0))
# Kinematic_Chain.setDH(
#     3, 0.0, 0.00, M_PI_2, 0.0, 1, to_rad(-110.), to_rad(110.), to_rad(120.0))
# Kinematic_Chain.setDH(
#     4, 0.0, 0.39, M_PI_2, 0.0, 1, to_rad(-170.), to_rad(170.), to_rad(140.0))
# Kinematic_Chain.setDH(
#     5, 0.0, 0.00, -M_PI_2, 0.0, 1, to_rad(-110.), to_rad(110.), to_rad(180.0))
# Kinematic_Chain.setDH(
#     6, 0.0, 0.126, 0.0, 0.0, 1, to_rad(-175.), to_rad(175.), to_rad(180.0))
# Kinematic_Chain.setTF(T0)
# Kinematic_Chain.readyForKinematics()
# print(Kinematic_Chain)