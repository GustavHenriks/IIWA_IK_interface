#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point32, TransformStamped
from std_msgs.msg import Header
import time
# from sensor_msgs.msg import PointCloud
# import geometry_msgs.msg


class convert_tf():
    def __init__(self):

        rospy.init_node('convert_tf', anonymous=True)

        listener = tf.TransformListener()
        br_svr = tf.TransformBroadcaster()
        br_ee = tf.TransformBroadcaster()
        br_ee_svr = tf.TransformBroadcaster()
        br_ee_conv = tf.TransformBroadcaster()
        br_ee_conv_debug = tf.TransformBroadcaster()
        br_robot_base_fixed =  tf.TransformBroadcaster()

        self.desired_end_received = False
        self.Hand_received = False

        self.RobotPosSub = rospy.Subscriber(
            "/IIWA/Real_E_Pos", Pose, self.chatterCallback_RobotEnd)
        self.GammaSub = rospy.Subscriber(
            "/gamma/pose", Pose, self.chatterCallback_Gamma)
        self.RobotPosConvPub = rospy.Publisher(
            "/robot/end/measured_converted", Pose, queue_size=3)
        self.RobotPosDesiredSub = rospy.Subscriber(
            "/robot/end/desired", Pose, self.chatterCallback_desiredPos)
        self.RobotPosDesiredConvertedPub = rospy.Publisher(
            "/robot/end/desired_converted", Pose, queue_size=3)

        self.end = Pose()
        self.Robot_conv = Pose()
        self.Robot_des = Pose()
        self.desired_end = Pose()

        rate = rospy.Rate(500.0)
        start = time.time()

        # calibration of the arm orientation
        delay=1
        while time.time() < start+delay and not rospy.is_shutdown():
            try:
                trans_arm = listener.lookupTransform(
                    '/mocap_hand', '/mocap_shoulder', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            p_arm = np.array(
                [trans_arm[0][0], trans_arm[0][1], trans_arm[0][2]])

            p_arm = p_arm

            br_svr.sendTransform(p_arm, trans_arm[1], rospy.Time.now(),
                                 'mocap_svr', "mocap_hand")


            # freezing the robot_base
            try:
                trans_world_base = listener.lookupTransform(
                    '/mocap_world', '/mocap_robot_base', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # assuming that the robot base is similar to the mocap_world (with pi around z-axis)

            q_world_base = tf.transformations.quaternion_about_axis(np.pi,(0,0,1))


            rate.sleep()

        print('Your '+str(delay)+' seconds is over!!!!')

        while not rospy.is_shutdown():

            br_robot_base_fixed.sendTransform(trans_world_base[0], q_world_base, rospy.Time.now(),
                                 'robot_base_fixed', "mocap_world")

            br_svr.sendTransform(p_arm, trans_arm[1], rospy.Time.now(),
                                 'mocap_svr', "mocap_hand")

            p_ee = [self.end.position.x,
                    self.end.position.y, self.end.position.z]
            q_ee = [self.end.orientation.x, self.end.orientation.y,
                    self.end.orientation.z, self.end.orientation.w]
            br_ee.sendTransform(p_ee, q_ee, rospy.Time.now(),
                                'mocap_ee', "robot_base_fixed")
            try:
                trans_end_svr = listener.lookupTransform(
                    '/mocap_svr', '/mocap_ee', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            try:
                trans_end_base = listener.lookupTransform(
                    '/robot_base_fixed', '/mocap_svr', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # we have gamma_vec in svr_frame, we need to transform it to the robot base for motion-planning
            # self.gamma_vec=np.array(trans_end_svr[0])*-1
            # self.gamma_vec=self.gamma_vec/np.linalg.norm(self.gamma_vec)
            if self.Hand_received:
                axis_y = np.array([0, 1, 0])
                axis_z = np.array(self.gamma_vec)*-1
                axis_z_on_y = np.dot(axis_y,axis_z)
                axis_z = axis_z - axis_z_on_y * axis_y 
                axis_z = axis_z/np.linalg.norm(axis_z)
                axis_x = np.cross(axis_y, axis_z)

                rot_mat = np.zeros((4, 4))
                rot_mat[:3, 0] = axis_x
                rot_mat[:3, 1] = axis_y
                rot_mat[:3, 2] = axis_z
                rot_mat[3, 3] = 1
                q_tf = tf.transformations.quaternion_from_matrix(rot_mat)

                self.publish_end_conv(np.array(trans_end_svr))

                br_ee_svr.sendTransform(trans_end_svr[0], q_tf, rospy.Time.now(),'e_desired_orientation', "mocap_svr")

                print(np.linalg.norm(self.desired_end_vec))
                print(np.linalg.norm(self.qv_mult(trans_end_base[1], self.desired_end_vec)))
                
                desired_end_base = trans_end_base[0] + self.qv_mult(trans_end_base[1], self.desired_end_vec)
                br_ee_conv.sendTransform(desired_end_base, tf.transformations.quaternion_multiply(trans_end_base[1],q_tf), rospy.Time.now(),'e_test', "robot_base_fixed")
                br_ee_conv_debug.sendTransform(self.desired_end_vec, tf.transformations.quaternion_about_axis(0,(1,0,0)), rospy.Time.now(),'e_debug', "mocap_svr")

            # print(self.desired_end_received)
            # Convert frame from SVR to base of robot
            # if self.desired_end_received:
                print(self.desired_end_vec)
                # print(desired_end_base)
                self.publish_end_desired(desired_end_base,tf.transformations.quaternion_multiply(trans_end_base[1],q_tf))
                # br_ee_conv.sendTransform(desired_end_base, tf.transformations.quaternion_multiply(trans_end_base[1],q_tf), rospy.Time.now(),'e_test2', "robot_base_fixed")
            rate.sleep()

    def chatterCallback_RobotEnd(self, data):
        self.end.position.x = data.position.x
        self.end.position.y = data.position.y
        self.end.position.z = data.position.z
        self.end.orientation.x = data.orientation.x
        self.end.orientation.y = data.orientation.y
        self.end.orientation.z = data.orientation.z
        self.end.orientation.w = data.orientation.w
        self.end_received = True

    def chatterCallback_desiredPos(self, data):
        self.desired_end_vec = np.array([data.position.x, data.position.y, data.position.z])
        self.desired_end_received = True

    def chatterCallback_Gamma(self, data):
        self.gamma_vec = [data.position.x, data.position.y, data.position.z]
        self.gamma_vec = self.gamma_vec/np.linalg.norm(self.gamma_vec)
        self.Hand_received = True

    def publish_end_conv(self, data):
        self.Robot_conv.position.x = data[0][0]
        self.Robot_conv.position.y = data[0][1]
        self.Robot_conv.position.z = data[0][2]
        self.Robot_conv.orientation.x = data[1][0]
        self.Robot_conv.orientation.y = data[1][1]
        self.Robot_conv.orientation.z = data[1][2]
        self.Robot_conv.orientation.w = data[1][3]
        self.RobotPosConvPub.publish(self.Robot_conv)

    def publish_end_desired(self, data, data2):
        self.Robot_des.position.x = data[0]
        self.Robot_des.position.y = data[1]
        self.Robot_des.position.z = data[2]
        self.Robot_des.orientation.x = data2[0]
        self.Robot_des.orientation.y = data2[1]
        self.Robot_des.orientation.z = data2[2]
        self.Robot_des.orientation.w = data2[3]
        self.RobotPosDesiredConvertedPub.publish(self.Robot_des)

    # rotate vector v1 by quaternion q1
    def qv_mult(self, q1, v1):
        v_norm = np.linalg.norm(v1)
        v1 = tf.transformations.unit_vector(v1)
        q2 = list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2),
            tf.transformations.quaternion_conjugate(q1)
        )[:3]*v_norm


if __name__ == '__main__':
    convert_tf()
