#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from geometry_msgs.msg import Pose


class mock_publisher():
    def __init__(self):
        print('test')
        rospy.init_node('mock_publisher', anonymous=True)
        self.endPub = rospy.Publisher(
            "/robot/end/measured", Pose, queue_size=3)
        self.headPub = rospy.Publisher(
            "/Head/pose", Pose, queue_size=3)
        self.basePub = rospy.Publisher(
            "/Base/pose", Pose, queue_size=3)
        self.endSub = rospy.Subscriber(
            "/robot/end/desired_converted", Pose, self.chatterCallback_desiredPos)
        self.init_end()
        self.init_head()
        self.init_base()
        print(self.head)
        self.desired_end_received = False
        r = rospy.Rate(300)
        while not rospy.is_shutdown():
            self.endPub.publish(self.end)
            self.basePub.publish(self.base)
            self.headPub.publish(self.head)
            if self.desired_end_received:
                self.update_end()
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

    def init_head(self):
        self.head = Pose()
        self.desired_head = Pose()
        self.head.position.x = 0.889693140984
        self.head.position.y = -1.05602824688
        self.head.position.z = 1.32516944408
        self.head.orientation.x = 0.00235101440921
        self.head.orientation.y = -0.00610731774941
        self.head.orientation.z = 0.0025682055857
        self.head.orientation.w = -0.999975264072

    def init_base(self):
        self.base = Pose()
        self.desired_base = Pose()
        self.base.position.x = -0.263918042183
        self.base.position.y = -0.961166739464
        self.base.position.z = 0.816419422626

    def update_end(self):
        self.end.position.x = self.desired_end.position.x
        self.end.position.y = self.desired_end.position.y
        self.end.position.z = self.desired_end.position.z
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
