#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Pose, PoseStamped, Point32
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud


class convert_frame():
    def __init__(self):
        self.init_params()
        self.load_pointcloud()
        self.init_nodes()
        # self.self_test()
        self.update()

    def update(self):
        r = rospy.Rate(1)
        # while not (self.end_received and self.Shoulder_received and self.base_received):
        #     r.sleep()
        r2 = rospy.Rate(500)
        print(self.base_received | self.Hand_received |
              self.Shoulder_received | self.end_received)
        while not (self.base_received | self.Hand_received | self.Shoulder_received | self.end_received):
            print('waiting for objects')
            r.sleep()
        print('All objects detected')
        print("End:", self.end)
        print("Base:", self.base)
        print("Shoulder:", self.Shoulder)
        print("Hand:", self.Hand)
        while not rospy.is_shutdown():
            self.convert_pos()
            self.RobotPosConvertedPub.publish(self.end_conv)
            self.publish_on_tf(self.end, 'Measured end')
            self.publish_on_tf(self.end_conv, 'Measured conv')
            self.publish_on_tf(self.Shoulder, 'Shoulder')
            self.publish_on_tf(self.Hand, 'Hand')
            self.publish_on_tf(self.base, 'Base')
            self.pubish_on_point_cloud(self.pointcloud)
            if self.desired_end_received:
                self.inv_convert_pos()
                self.convert_orientation()
                # print(self.desired_end_conv)
                self.RobotPosDesiredConvertedPub.publish(self.desired_end_conv)
                self.publish_on_tf(self.desired_end, 'desired_end')
                self.publish_on_tf(self.desired_end_conv, 'desired_end_conv')
            r2.sleep()

    def init_params(self):
        self.end = Pose()
        self.end_conv = Pose()
        self.desired_end = Pose()
        self.desired_end_conv = Pose()
        self.Shoulder = Pose()
        self.Hand = Pose()
        self.base = Pose()
        self.end_received = False
        self.base_received = False
        self.Shoulder_received = False
        self.Hand_received = False
        self.desired_end_received = False
        self.svm_dir = [0, 1, 0]
        self.up = [0, 0, 1]
        self.desired_end_vec_conv = [0, 0, 0]
        self.rot_mat = np.zeros((4, 4))
        self.rotation = np.zeros((3, 3))
        self.dir = [1, 0, 1]
        self.gamma_vec = [0, 0, 0]

    def init_nodes(self):
        rospy.init_node('convert_frame', anonymous=True)
        # self.RobotPosSub = rospy.Subscriber(
        #     "/robot/end/measured", Pose, self.chatterCallback_RobotEnd)
        self.RobotPosSub = rospy.Subscriber(
            "/IIWA/Real_E_Pos", Pose, self.chatterCallback_RobotEnd)
        self.RobotPosSub = rospy.Subscriber(
            "/Robot_base/pose", PoseStamped, self.chatterCallback_RobotBase)
        self.RobotPosSub = rospy.Subscriber(
            "/Shoulder/pose", PoseStamped, self.chatterCallback_Shoulder)
        self.RobotPosSub = rospy.Subscriber(
            "/Hand/pose", PoseStamped, self.chatterCallback_Hand)
        self.RobotPosDesiredSub = rospy.Subscriber(
            "/robot/end/desired", Pose, self.chatterCallback_desiredPos)
        self.RobotPosConvertedPub = rospy.Publisher(
            "/robot/end/measured_converted", Pose, queue_size=3)
        self.RobotPosDesiredConvertedPub = rospy.Publisher(
            "/robot/end/desired_converted", Pose, queue_size=3)
        self.GammaSub = rospy.Subscriber(
            "/gamma/pose", Pose, self.chatterCallback_Gamma)
        self.CloudPub = rospy.Publisher(
            "/PointCloud/points", PointCloud)
        # self.RobotPosDesiredConvertedPub = rospy.Publisher(
        #     "/IIWA/Desired_E_Pos", Pose, queue_size=3)

    def convert_pos(self):
        self.end_vec = np.array([self.end.position.x,
                                 self.end.position.y, self.end.position.z])
        self.base_vec = np.array([self.base.position.x,
                                  self.base.position.y, self.base.position.z])*[-1, -1, 1]
        self.Shoulder_vec = np.array([self.Shoulder.position.x,
                                      self.Shoulder.position.y, self.Shoulder.position.z])*[-1, -1, 1]
        self.Hand_vec = np.array(
            [self.Hand.position.x, self.Hand.position.y, self.Hand.position.z])*[-1, -1, 1]
        self.Shoulder_o = np.array([self.Shoulder.orientation.x, self.Shoulder.orientation.y,
                                    self.Shoulder.orientation.z, self.Shoulder.orientation.w])
        self.Hand_vec = self.Hand_vec-self.base_vec
        self.Shoulder_vec = self.Shoulder_vec - self.base_vec
        self.base_vec = self.base_vec - self.base_vec

        self.Hand_vec = self.Hand_vec - self.Shoulder_vec
        self.end_vec = self.end_vec - self.Shoulder_vec
        self.base_vec = self.base_vec - self.Shoulder_vec
        self.Shoulder_vec = self.Shoulder_vec - self.Shoulder_vec

        arm_dir = (self.Hand_vec - self.Shoulder_vec) / \
            np.linalg.norm((self.Hand_vec - self.Shoulder_vec))
        self.rotation = self.rot(arm_dir, self.svm_dir)
        self.end_vec_conv = np.dot(self.rotation, self.end_vec.transpose())

        self.end_conv.position.x = self.end_vec_conv[0]
        self.end_conv.position.y = self.end_vec_conv[1]
        self.end_conv.position.z = self.end_vec_conv[2]
        # print(self.end_vec)
        # print(self.end_conv)
        # raw_input('..')
        # Hand=Hand-Robot
        # End=End
        # Shoulder=Shoulder-Robot
        # Robot=Robot-Robot

        # Hand=Hand-Shoulder
        # End=End-Shoulder
        # Robot=Robot-Shoulder
        # Shoulder=Shoulder-Shoulder

        # svm_dir=[0,1,0];
        # arm_dir=[0,1,0.000000001];
        # % arm_dir=(Hand-Shoulder)/norm(Hand-Shoulder);
        # W=cross(svm_dir,arm_dir)
        # A= [svm_dir',W',cross(svm_dir,W)']
        # B= [arm_dir',W',cross(arm_dir,W)']
        # rot=B*inv(A);
        # Xs=(rot*Xs')'

        # self.tran = tf.transformations.quaternion_matrix(self.Shoulder_o)
        # self.tran[0, 3] = self.base_vec[0]
        # self.tran[1, 3] = self.base_vec[1]
        # self.tran[2, 3] = self.base_vec[2]
        # self.end_vec = np.append(self.end_vec, np.array([1]))
        # self.end_vec_conv = np.matmul(self.tran, self.end_vec)

        # self.rotation[0:3, 0:3] = self.rot(
        #     [0, 1, 0], self.Shoulder_vec-self.Hand_vec)
        # self.rotation[3, 3] = 1
        # self.quat_mat = tf.transformations.quaternion_matrix(
        #     tf.transformations.quaternion_from_matrix(self.rotation))
        # # print(np.array([self.end_vec_conv[0],self.end_vec_conv[1],self.end_vec_conv[2], 1]))
        # self.end_vec_conv = np.dot(self.quat_mat, np.array(
        #     [self.end_vec_conv[0], self.end_vec_conv[1], self.end_vec_conv[2], 1]).transpose())

        # self.end_conv.position.x = self.end_vec_conv[0]
        # self.end_conv.position.y = self.end_vec_conv[1]
        # self.end_conv.position.z = self.end_vec_conv[2]

    def inv_convert_pos(self):
        self.desired_end_vec_conv_prev = self.desired_end_vec_conv
        self.desired_end_vec = np.array([self.desired_end.position.x,
                                         self.desired_end.position.y, self.desired_end.position.z])
        self.base_vec = np.array([self.base.position.x,
                                  self.base.position.y, self.base.position.z])*[-1, -1, 1]
        self.Shoulder_vec = np.array([self.Shoulder.position.x,
                                      self.Shoulder.position.y, self.Shoulder.position.z])*[-1, -1, 1]
        self.Hand_vec = np.array(
            [self.Hand.position.x, self.Hand.position.y, self.Hand.position.z])*[-1, -1, 1]
        self.Shoulder_o = np.array([self.Shoulder.orientation.x, self.Shoulder.orientation.y,
                                    self.Shoulder.orientation.z, self.Shoulder.orientation.w])
        self.Hand_vec = self.Hand_vec-self.base_vec
        self.Shoulder_vec2 = self.Shoulder_vec - self.base_vec
        self.base_vec = self.base_vec - self.base_vec

        self.Hand_vec = self.Hand_vec - self.Shoulder_vec2
        # self.desired_end_vec = self.desired_end_vec - self.Shoulder_vec2?
        self.base_vec = self.base_vec - self.Shoulder_vec2
        self.Shoulder_vec = self.Shoulder_vec2 - self.Shoulder_vec2

        arm_dir = (self.Hand_vec - self.Shoulder_vec) / \
            np.linalg.norm((self.Hand_vec - self.Shoulder_vec))
        self.rotation = self.rot(arm_dir, self.svm_dir)
        self.rotation2 = np.linalg.inv(self.rotation)
        self.desired_end_vec_conv = np.dot(
            self.rotation2, self.desired_end_vec.transpose())

        self.desired_end_vec_conv = self.desired_end_vec_conv + self.Shoulder_vec2

        self.desired_end_conv.position.x = self.desired_end_vec_conv[0]
        self.desired_end_conv.position.y = self.desired_end_vec_conv[1]
        self.desired_end_conv.position.z = self.desired_end_vec_conv[2]
        # self.desired_end_conv.position.x = -0.509373647132
        # self.desired_end_conv.position.y = -0.00988616389521
        # self.desired_end_conv.position.z = 0.797298121645
        # print(self.desired_end_vec_conv)
        # self.desired_end_vec = np.array([self.desired_end.position.x,
        #                                  self.desired_end.position.y, self.desired_end.position.z])
        # self.base_vec = np.array([self.base.position.x,
        #                           self.base.position.y, self.base.position.z])
        # self.Shoulder_vec = np.array([self.Shoulder.position.x,
        #                               self.Shoulder.position.y, self.Shoulder.position.z])
        # self.Shoulder_o = np.array([self.Shoulder.orientation.x, self.Shoulder.orientation.y,
        #                             self.Shoulder.orientation.z, self.Shoulder.orientation.w])
        # self.rotation[0:3, 0:3] = self.rot(
        #     self.Shoulder_vec-self.Hand_vec, [0, 1, 0])
        # self.quat_mat = tf.transformations.quaternion_matrix(
        #     tf.transformations.quaternion_from_matrix(self.rotation))
        # self.end_vec_conv = np.dot(self.quat_mat, np.array(
        #     [self.end_vec_conv[0], self.end_vec_conv[1], self.end_vec_conv[2], 1]).transpose())

        # self.base_vec = self.base_vec - self.Shoulder_vec
        # self.Shoulder_vec = self.Shoulder_vec - self.Shoulder_vec
        # self.tran = tf.transformations.quaternion_matrix(self.Shoulder_o)
        # self.tran[0, 3] = self.base_vec[0]
        # self.tran[1, 3] = self.base_vec[1]
        # self.tran[2, 3] = self.base_vec[2]
        # self.desired_end_vec = np.append(self.desired_end_vec, np.array([1]))
        # self.desired_end_vec_conv = np.matmul(
        #     np.linalg.inv(self.tran), self.desired_end_vec)

        # self.desired_end_conv.position.x = -self.desired_end_vec_conv[0]
        # self.desired_end_conv.position.y = -self.desired_end_vec_conv[1]
        # self.desired_end_conv.position.z = self.desired_end_vec_conv[2]

    def convert_orientation(self):
        # print(self.desired_end_conv.position.x-self.end.position.x)
        # print(self.desired_end_conv.position.z-self.end.position.y)
        # print(self.desired_end_conv.position.z-self.end.position.z)
        # self.dirx = self.desired_end_conv.position.x-self.end.position.x
        # self.diry = self.desired_end_conv.position.y-self.end.position.y
        # self.dirz = self.desired_end_conv.position.z-self.end.position.z
        self.dirx = self.gamma_vec[0]
        self.diry = self.gamma_vec[1]
        self.dirz = self.gamma_vec[2]
        # print(self.gamma_vec)
        # print(self.desired_end_vec_conv)
        # print(self.desired_end_vec_conv_prev)
        # print(self.desired_end_vec_conv-self.desired_end_vec_conv_prev)
        # if np.count_nonzero(self.desired_end_vec_conv-self.desired_end_vec_conv_prev)!=3:
        #     self.dir = (self.desired_end_vec_conv-self.desired_end_vec_conv_prev)/np.linalg.norm(self.desired_end_vec_conv-self.desired_end_vec_conv_prev)
        # self.diry = self.desired_end.position.x-self.end_conv.position.x
        # self.dirx = self.desired_end.position.y-self.end_conv.position.y
        # self.dirz = self.desired_end.position.z-self.end_conv.position.z

        self.dir = [self.dirx, self.diry, self.dirz] / \
            np.linalg.norm([self.dirx, self.diry, self.dirz])
        # print("Dir", self.dir)
        self.cross_vec1 = np.cross(
            self.up, self.dir)/np.linalg.norm(np.cross(self.up, self.dir))
        # print("cross_Vec1 ", self.cross_vec1)
        self.cross_vec1 = (self.Hand_vec-self.Shoulder_vec) / \
            np.linalg.norm(self.Hand_vec-self.Shoulder_vec)
        self.cross_vec2 = np.cross(
            self.dir, self.cross_vec1)/np.linalg.norm(np.cross(self.dir, self.cross_vec1))
        # print("cross_Vec2 ", self.cross_vec2)
        self.rot_mat[0:3, 0] = self.dir
        self.rot_mat[0:3, 1] = self.cross_vec1
        self.rot_mat[0:3, 2] = self.cross_vec2
        # self.rot_mat[0, 0:3] = self.cross_vec1
        # self.rot_mat[1, 0:3] = self.cross_vec2
        # self.rot_mat[2, 0:3] = self.dir
        self.rot_mat[3, 3] = 1
        # print("Rot_mat ", self.rot_mat)

        # self.desired_end_rotation = tf.transformations.quaternion_matrix(
        #     [self.desired_end_conv.orientation.x, self.desired_end_conv.orientation.y, self.desired_end_conv.orientation.z, self.desired_end_conv.orientation.w])
        # self.quat_tmp = tf.transformations.quaternion_from_matrix(
        #     np.dot(self.rot_mat, self.desired_end_rotation))

        self.rot_mat = np.dot(
            tf.transformations.rotation_matrix(-90, (0, 1, 0)), self.rot_mat)
        self.quat_tmp = tf.transformations.quaternion_from_matrix(self.rot_mat)
        # print("quat_tmp", self.quat_tmp )
        self.desired_end_conv.orientation.x = self.quat_tmp[0]
        self.desired_end_conv.orientation.y = self.quat_tmp[1]
        self.desired_end_conv.orientation.z = self.quat_tmp[2]
        self.desired_end_conv.orientation.w = self.quat_tmp[3]
        # self.desired_end_conv.orientation.x = -0.0322475021333
        # self.desired_end_conv.orientation.y = 0.937675324723
        # self.desired_end_conv.orientation.z =  0.00206473094424
        # self.desired_end_conv.orientation.w = -0.346006966545
        # print(self.dir)

    def self_test(self):
        self.base_o = [-0.000830705626868, 0.000430435611634,
                       0.00137064396404, -0.999998688698]
        self.base_p = np.array(
            [-0.263918042183,  -0.961166739464, 0.816419422626])
        self.end_p = np.array(
            [-1.03592442281,  0.0802183864893, 0.526110662425])
        self.Shoulder_p = np.array(
            [0.889693140984, -1.05602824688, 1.32516944408])
        self.Shoulder_o = np.array(
            [-0.000830705626868, 0.000430435611634, 0.00137064396404, -0.999998688698])
        self.base_p = self.base_p - self.Shoulder_p
        self.Shoulder_p = self.Shoulder_p - self.Shoulder_p
        self.base_p = np.append(self.base_p, 1)
        self.Shoulder_p = np.append(self.Shoulder_p, np.array([1]))
        self.end_p = np.append(self.end_p, np.array([1]))
        self.end_p[0] = -self.end_p[0]
        self.end_p[1] = -self.end_p[1]
        self.tran = tf.transformations.quaternion_matrix(self.Shoulder_o)
        self.tran[0, 3] = self.base_p[0]
        self.tran[1, 3] = self.base_p[1]
        self.tran[2, 3] = self.base_p[2]
        print(self.base_p)
        print(self.Shoulder_p)
        print(self.end_p)
        print(self.tran)
        print(self.Shoulder_o)
        print(tf.transformations.quaternion_from_matrix(self.tran))
        print(np.matmul(self.tran, self.end_p))
        print(tf.transformations.rotation_matrix(0.4, [1, 2, 3]))

    def chatterCallback_desiredPos(self, data):
        self.desired_end.position.x = data.position.x
        self.desired_end.position.y = data.position.y
        self.desired_end.position.z = data.position.z
        self.desired_end.orientation.x = data.orientation.x
        self.desired_end.orientation.y = data.orientation.y
        self.desired_end.orientation.z = data.orientation.z
        self.desired_end.orientation.w = data.orientation.w
        self.desired_end_received = True

    def chatterCallback_RobotEnd(self, data):
        self.end.position.x = data.position.x
        self.end.position.y = data.position.y
        self.end.position.z = data.position.z
        self.end.orientation.x = data.orientation.x
        self.end.orientation.y = data.orientation.y
        self.end.orientation.z = data.orientation.z
        self.end.orientation.w = data.orientation.w
        self.end_received = True

    def chatterCallback_RobotBase(self, data):
        self.base.position.x = data.pose.position.x
        self.base.position.y = data.pose.position.y
        self.base.position.z = data.pose.position.z
        self.base.orientation.x = data.pose.orientation.x
        self.base.orientation.y = data.pose.orientation.y
        self.base.orientation.z = data.pose.orientation.z
        self.base.orientation.w = data.pose.orientation.w
        self.base_received = True

    def chatterCallback_Shoulder(self, data):
        self.Shoulder.position.x = data.pose.position.x+0.155
        self.Shoulder.position.y = data.pose.position.y
        self.Shoulder.position.z = data.pose.position.z-0.03-0.005
        self.Shoulder.orientation.x = data.pose.orientation.x
        self.Shoulder.orientation.y = data.pose.orientation.y
        self.Shoulder.orientation.z = data.pose.orientation.z
        self.Shoulder.orientation.w = data.pose.orientation.w
        self.Shoulder_received = True

    def chatterCallback_Hand(self, data):
        self.Hand.position.x = data.pose.position.x+0.01+0.155
        self.Hand.position.y = data.pose.position.y
        self.Hand.position.z = data.pose.position.z-0.005
        self.Hand.orientation.x = data.pose.orientation.x
        self.Hand.orientation.y = data.pose.orientation.y
        self.Hand.orientation.z = data.pose.orientation.z
        self.Hand.orientation.w = data.pose.orientation.w
        self.Hand_received = True

    def chatterCallback_Gamma(self, data):
        self.gamma_vec = [data.position.x, data.position.y, data.position.z]
        self.Hand_received = True

    def rot(self, U, V):
        # print(U)
        # print(V)
        W = np.cross(U, V)
        A = np.array([U, W, np.cross(U, W)])
        B = np.array([V, W, np.cross(V, W)])
        return np.dot(B, np.linalg.inv(A))

    def publish_on_tf(self, Obj, name):
        br = tf.TransformBroadcaster()
        Position = [Obj.position.x, Obj.position.y, Obj.position.z]
        if name == 'Shoulder':
            Position = (np.array(Position)-np.array(Position))*[-1, -1, 1]
        if name == 'Hand':
            # print(Position)
            Position = np.array(
                Position)-np.array([self.base.position.x, self.base.position.y, self.base.position.z])
            # print(Position)
            Shoulder = np.array([self.Shoulder.position.x, self.Shoulder.position.y, self.Shoulder.position.z]
                                )-np.array([self.base.position.x, self.base.position.y, self.base.position.z])
            Position = (Position - Shoulder)*[-1, -1, 1]
            # print(Shoulder)
            # print(Position*[-1,-1,1])
            # raw_input()
        if name == 'Base':
            Position = np.array(
                Position)-np.array([self.base.position.x, self.base.position.y, self.base.position.z])
            Shoulder = np.array([self.Shoulder.position.x, self.Shoulder.position.y, self.Shoulder.position.z]
                                )-np.array([self.base.position.x, self.base.position.y, self.base.position.z])
            Position = (Position - Shoulder)*[-1, -1, 1]
        if name == 'desired_end':
            Position = np.array(Position)

        Rotation = tf.transformations.quaternion_matrix(
            [Obj.orientation.x, Obj.orientation.y, Obj.orientation.z, Obj.orientation.w])
        br.sendTransform((Position[0], Position[1], Position[2]), tf.transformations.quaternion_from_matrix(
            Rotation), rospy.Time.now(), name, "world_frame")

    def pubish_on_point_cloud(self, X):
        pointCloud = PointCloud()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world_frame'
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
            "/home/gustavhenriks/catkin_ws_ik_test/src/IIWA_IK_interface/iiwa_scenarios/scripts/data/Pointcloud/pointcloud")


if __name__ == '__main__':
    convert_frame()
