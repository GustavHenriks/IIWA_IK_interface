#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point32, TransformStamped
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud
import time
# from sensor_msgs.msg import PointCloud
# import geometry_msgs.msg


class convert_tf():
    def __init__(self):

        freq = 500
        rospy.init_node('convert_tf', anonymous=True)

        self.listener = tf.TransformListener()
        br_svr = tf.TransformBroadcaster()
        br_ee = tf.TransformBroadcaster()
        br_ee_svr = tf.TransformBroadcaster()
        br_ee_conv = tf.TransformBroadcaster()
        br_ee_conv_debug = tf.TransformBroadcaster()
        br_robot_base_fixed = tf.TransformBroadcaster()
        br_svr_rotated = tf.TransformBroadcaster()
        br_hand_filter = tf.TransformBroadcaster()
        br_hand_robot = tf.TransformBroadcaster()
        br_shoudler_robot = tf.TransformBroadcaster()

        self.desired_end_received = False
        self.Hand_received = False

        self.RobotPosSub = rospy.Subscriber(
            "/IIWA/Real_E_Pos", Pose, self.chatterCallback_RobotEnd)
        self.RobotHandPub = rospy.Publisher(
            "/Hand_robot/pose", PoseStamped, queue_size=3)
        self.RobotShoulderPub = rospy.Publisher(
            "/Shoulder_robot/pose", PoseStamped, queue_size=3)
        self.GammaSub = rospy.Subscriber(
            "/gamma/pose", Pose, self.chatterCallback_Gamma)
        self.RobotPosConvPub = rospy.Publisher(
            "/robot/end/measured_converted", Pose, queue_size=3)
        self.RobotPosDesiredSub = rospy.Subscriber(
            "/robot/end/desired", Pose, self.chatterCallback_desiredPos)
        self.RobotPosDesiredConvertedPub = rospy.Publisher(
            "/robot/end/desired_converted", Pose, queue_size=3)
        self.CloudPub = rospy.Publisher(
            "/PointCloud/points", PointCloud)

        self.end = Pose()
        self.Robot_conv = Pose()
        self.Robot_des = Pose()
        self.desired_end = Pose()
        self.trans_arm = []
        self.Robot_hand = PoseStamped()
        self.Robot_shoulder = PoseStamped()
        rate = rospy.Rate(freq)
        start = time.time()

        # calibration of the arm orientation
        print('Calibrating the arm orientation')
        delay = 1
        while time.time() < start+delay and not rospy.is_shutdown():
            try:
                trans_arm = self.listener.lookupTransform(
                    '/mocap_hand', '/mocap_shoulder', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.trans_arm = trans_arm
            p_arm = np.array(
                [trans_arm[0][0], trans_arm[0][1], trans_arm[0][2]])

            p_arm = p_arm/1.7
            # print(p_arm, trans_arm[1])

            br_svr.sendTransform(p_arm, trans_arm[1], rospy.Time.now(),
                                 'mocap_svr', "mocap_hand")

            # freezing the robot_base
            try:
                trans_world_base = self.listener.lookupTransform(
                    '/mocap_world', '/mocap_robot_base', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # assuming that the robot base is similar to the mocap_world (with pi around z-axis)

            # q_world_base = tf.transformations.quaternion_about_axis(
            #     np.pi, (0, 0, 1))

            # Use for weird mocap config
            q_world_base = tf.transformations.quaternion_about_axis(
                np.pi/2, (0, 0, 1))

            rate.sleep()

        print('Calibration done')
        print('Waiting for robot..')
        while not self.Hand_received:
            rate.sleep()
        print('Robot reached')
        print('Commencing frame transformation')
        p_hand_filtered = np.array([0, 0, 0])
        q_hand_filtered = np.array([0, 0, 0, 0])
        filter_factor = 1 - (1.0/freq) / (1.0/freq + 1.0/10)
        while not rospy.is_shutdown():

            br_robot_base_fixed.sendTransform(trans_world_base[0], q_world_base, rospy.Time.now(),
                                 'robot_base_fixed', "mocap_world")

            # filtering the mocap_hand frame
            try:
                trans_hand = self.listener.lookupTransform(
                    '/mocap_world', '/mocap_hand', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            # print(trans_hand[1])
            np_trans_0 = np.array(trans_hand[0])
            np_trans_1 = np.array(trans_hand[1])
            # print(np_trans_1)
            p_hand_filtered = (1-filter_factor) * \
                               p_hand_filtered + filter_factor * np_trans_0
            q_hand_filtered = (1-filter_factor) * \
                               q_hand_filtered + filter_factor * np_trans_1
            # print(q_hand_filtered)
            q_hand_filtered = q_hand_filtered / np.linalg.norm(q_hand_filtered)

            br_hand_filter.sendTransform(p_hand_filtered, q_hand_filtered, rospy.Time.now(),
                    'mocap_hand_filtered', "mocap_world")

            br_svr.sendTransform(p_arm, trans_arm[1], rospy.Time.now(),
                                 'mocap_svr', "mocap_hand_filtered")

            p_ee = [self.end.position.x,
                    self.end.position.y, self.end.position.z]
            q_ee = [self.end.orientation.x, self.end.orientation.y,
                    self.end.orientation.z, self.end.orientation.w]
            br_ee.sendTransform(p_ee, q_ee, rospy.Time.now(),
                                'mocap_ee', "robot_base_fixed")

            # q_svr_rot = tf.transformations.quaternion_about_axis(-np.pi/12,(1,0,0))
            q_svr_rot = tf.transformations.quaternion_about_axis(
                np.deg2rad(3), (1, 0, 0))
            q_svr_rot = tf.transformations.quaternion_multiply(
                q_svr_rot, trans_arm[1])
            q_svr_rot = tf.transformations.quaternion_multiply(
                tf.transformations.quaternion_about_axis(np.deg2rad(90), (0, 0, 1)), q_svr_rot)
            br_svr_rotated.sendTransform(p_arm-[0.0, -0.01, 0.01], q_svr_rot, rospy.Time.now(),
                                 'mocap_svr_rotated', "mocap_hand_filtered")

            try:
                trans_end_base = self.listener.lookupTransform(
                    '/robot_base_fixed', '/mocap_svr_rotated', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            try:
                trans_end_svr = self.listener.lookupTransform(
                    '/mocap_svr_rotated', '/mocap_ee', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.trans_end_svr = trans_end_svr

            # we have gamma_vec in svr_frame, we need to transform it to the robot base for motion-planning
            # self.gamma_vec=np.array(trans_end_svr[0])*-1
            # self.gamma_vec=self.gamma_vec/np.linalg.norm(self.gamma_vec)
            if self.Hand_received:
                axis_y = np.array([0, 1, 0])
                axis_z = np.array(self.gamma_vec)*-1
                axis_z_on_y = np.dot(axis_y, axis_z)
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

                br_ee_svr.sendTransform(trans_end_svr[0], q_tf, rospy.Time.now(
                ), 'e_desired_orientation', "mocap_svr_rotated")

                # print(np.linalg.norm(self.desired_end_vec))
                # print(np.linalg.norm(self.qv_mult(trans_end_base[1], self.desired_end_vec)))

                desired_end_base = trans_end_base[0] + self.qv_mult(
                    trans_end_base[1], self.desired_end_vec)
                br_ee_conv.sendTransform(desired_end_base, tf.transformations.quaternion_multiply(
                    trans_end_base[1], q_tf), rospy.Time.now(), 'e_test', "robot_base_fixed")
                br_ee_conv_debug.sendTransform(self.desired_end_vec, tf.transformations.quaternion_about_axis(
                    0, (1, 0, 0)), rospy.Time.now(), 'e_debug', "mocap_svr_rotated")

            # print(self.desired_end_received)
            # Convert frame from SVR to base of robot
            # if self.desired_end_received:
                # print(self.desired_end_vec)
                # print(desired_end_base)
                self.publish_end_desired(
                    desired_end_base, tf.transformations.quaternion_multiply(trans_end_base[1], q_tf))
                # br_ee_conv.sendTransform(desired_end_base, tf.transformations.quaternion_multiply(trans_end_base[1],q_tf), rospy.Time.now(),'e_test2', "robot_base_fixed")
            self.load_pointcloud()
            self.pubish_on_point_cloud(self.pointcloud)
            self.publish_arm_and_shoulder()
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
        self.desired_end_vec = np.array(
            [data.position.x, data.position.y, data.position.z])
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

    def pubish_on_point_cloud(self, X):
        pointCloud = PointCloud()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'mocap_svr_rotated'
        pointCloud.header = header
        pointCloud.points = []
        for i in range(len(X)):
            Point_temp = Point32()
            Point_temp.x = X[i][0]
            Point_temp.y = X[i][1]
            Point_temp.z = X[i][2]

            pointCloud.points.append(Point_temp)
        self.CloudPub.publish(pointCloud)

    def load_pointcloud(self):
        self.pointcloud = np.loadtxt(
            "/home/gustavhenriks/catkin_ws_ik_test/src/IIWA_IK_interface/iiwa_scenarios/scripts/data/Pointcloud/pointcloud3.txt")

    def publish_arm_and_shoulder(self):
        try:
            # trans_end_hand = self.listener.lookupTransform(
            #     '/robot_base_fixed', '/mocap_hand_filtered', rospy.Time(0))
            trans_end_hand = self.listener.lookupTransform(
                '/mocap_svr_rotated', '/mocap_hand_filtered', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No success transforming SVR to Hand")
        self.Robot_hand.pose.position.x=trans_end_hand[0][0]
        self.Robot_hand.pose.position.y=trans_end_hand[0][1]
        self.Robot_hand.pose.position.z=trans_end_hand[0][2]
        self.Robot_hand.pose.orientation.x=trans_end_hand[1][0]
        self.Robot_hand.pose.orientation.y=trans_end_hand[1][1]
        self.Robot_hand.pose.orientation.z=trans_end_hand[1][2]
        self.Robot_hand.pose.orientation.w=trans_end_hand[1][3]
        self.RobotHandPub.publish(self.Robot_hand)

        try:
            # trans_end_shoulder = self.listener.lookupTransform(
            #     '/robot_base_fixed', '/mocap_svr_rotated', rospy.Time(0))
            trans_end_shoulder = self.listener.lookupTransform(
                '/mocap_svr_rotated', '/mocap_svr_rotated', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No success transforming SVR to SVR")
        self.Robot_shoulder.pose.position.x=trans_end_shoulder[0][0]
        self.Robot_shoulder.pose.position.y=trans_end_shoulder[0][1]
        self.Robot_shoulder.pose.position.z=trans_end_shoulder[0][2]
        self.Robot_shoulder.pose.orientation.x=trans_end_shoulder[1][0]
        self.Robot_shoulder.pose.orientation.y=trans_end_shoulder[1][1]
        self.Robot_shoulder.pose.orientation.z=trans_end_shoulder[1][2]
        self.Robot_shoulder.pose.orientation.w=trans_end_shoulder[1][3]
        self.RobotShoulderPub.publish(self.Robot_shoulder)

if __name__ == '__main__':
    convert_tf()
