#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Pose, PoseStamped

dt = 0.001


class mock_publisher():
    def __init__(self):
        print('test')
        rospy.init_node('mock_publisher', anonymous=True)
        # self.endPub = rospy.Publisher(
        #     "/IIWA/Real_E_Pos", Pose, queue_size=3)
        self.shoulderPub = rospy.Publisher(
            "/Shoulder/pose", PoseStamped, queue_size=3)
        self.handPub = rospy.Publisher(
            "/Hand/pose", PoseStamped, queue_size=3)
        self.basePub = rospy.Publisher(
            "/Robot_base/pose", PoseStamped, queue_size=3)
        self.endSub = rospy.Subscriber(
            "/robot/end/desired_converted", Pose, self.chatterCallback_desiredPos)
        # self.endSub = rospy.Subscriber(
        #     "/IIWA/Desired_E_Pos", Pose, self.chatterCallback_desiredPos)
        self.init_end()
        self.init_hand()
        self.init_shoulder()
        self.init_base()
        print(self.shoulder)
        self.desired_end_received = False
        r = rospy.Rate(300)
        while not rospy.is_shutdown():
            # self.endPub.publish(self.end)
            self.basePub.publish(self.base)
            self.shoulderPub.publish(self.shoulder)
            self.handPub.publish(self.hand)
            # if self.desired_end_received:
            #     self.update_end()
            r.sleep()

    def init_end(self):
        self.end = Pose()
        self.desired_end = Pose()
        # self.end.position.x = -1.03592442281
        # self.end.position.y = 0.0802183864893
        # self.end.position.z = 0.526110662425
        self.end.position.x = -0.5
        self.end.position.y = 0.0802183864893
        self.end.position.z = 1.526110662425

    def init_shoulder(self):
        self.shoulder = PoseStamped()
        self.desired_shoulder = Pose()
        self.shoulder.pose.position.x = 0.527764558792-0.20
        self.shoulder.pose.position.y = -0.618398308754
        self.shoulder.pose.position.z = 1.22047162056+0.05
        self.shoulder.pose.orientation.x = -0.664777219296
        self.shoulder.pose.orientation.y = 0.0852108821273
        self.shoulder.pose.orientation.z = -0.130120888352
        self.shoulder.pose.orientation.w = -0.730670273304

    def init_hand(self):
        self.hand = PoseStamped()
        self.desired_hand = Pose()
        self.hand.pose.position.x = 0.467085987329-0.23
        self.hand.pose.position.y = -1.19549167156
        self.hand.pose.position.z = 1.1878027916+0.1
        self.hand.pose.orientation.x = -0.00212644506246
        self.hand.pose.orientation.y = 0.00170458073262
        self.hand.pose.orientation.z = -0.0156762357801
        self.hand.pose.orientation.w = -0.999873459339

    def init_base(self):
        self.base = PoseStamped()
        self.desired_base = Pose()
        self.base.pose.position.x = -0.263918042183
        self.base.pose.position.y = -0.961166739464
        self.base.pose.position.z = 0.816419422626

    def update_end(self):
        self.end.position.x = self.end.position.x + \
            (self.desired_end.position.x-self.end.position.x)*dt
        self.end.position.y = self.end.position.y + \
            (self.desired_end.position.y-self.end.position.y)*dt
        self.end.position.z = self.end.position.z + \
            (self.desired_end.position.z-self.end.position.z)*dt
        # self.end.orientation.x = self.desired_end.orientation.x
        # self.end.orientation.y = self.desired_end.orientation.y
        # self.end.orientation.z = self.desired_end.orientation.z
        # self.end.orientation.w = self.desired_end.orientation.w

    def chatterCallback_desiredPos(self, data):
        self.desired_end.position.x = data.position.x
        self.desired_end.position.y = data.position.y
        self.desired_end.position.z = data.position.z
        self.desired_end_received = True


if __name__ == '__main__':
    mock_publisher()
