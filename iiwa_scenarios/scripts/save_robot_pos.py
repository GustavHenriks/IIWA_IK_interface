#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import numpy as np
import time

out_vec = []

def main():
    rospy.init_node("get_robot_pos", anonymous=True)
    r = rospy.Rate(500)
    rospy.Subscriber("/IIWA/Real_E_Pos", Pose, callback)
    print('Recording...')
    while  not(rospy.is_shutdown()):
        r.sleep()
    rospy.on_shutdown(myhook()) 


def callback(pos):
    out_vec.append([pos.position.x, pos.position.y, pos.position.z])
    # print(out_vec)
        
def myhook():
    # print(len(effort_arr), len(joint_arr), len(force_arr))
    # force_arr = force_arr - offset
    t = time.time()
    np.save('/home/gustavhenriks/catkin_ws_ik_test/src/IIWA_IK_interface/iiwa_scenarios/scripts/data/Rec_' + str(t) + '.npy',out_vec)
    print('Number of points : ', len(out_vec))
main()