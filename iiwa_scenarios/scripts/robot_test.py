#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
import tf.transformations as t
import tf
from math import pi
import numpy as np

cur_pos = []
step = 0.02
X_s = np.load('/home/gustavhenriks/catkin_ws_ik_test/src/IIWA_IK_interface/iiwa_scenarios/scripts/s_shape.npy')
count = 0
# print(X_s)

def main():
    rospy.init_node("read_position", anonymous=True)
    run_pos()

def callback(data):
    cur_pos.append([data.position.x, data.position.y, data.position.z,
                    data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w ])   

def to_rad(x):
        return(x/180 * pi)


def target_pos():
    pos = Pose()
    changed = []
    last_position = cur_pos[-1]
#     pos.position.x = last_position[0] 
#     pos.position.y = last_position[1]
#     pos.position.z = last_position[2]
    pos.position.x = -0.727346501546
    pos.position.y = -0.205394142472
    pos.position.z = 0.569382580131
    pos.orientation.x = last_position[3] 
    pos.orientation.y = last_position[4] 
    pos.orientation.z = last_position[5]
    pos.orientation.w = last_position[6] 
    quaternion = (
        pos.orientation.x,
        pos.orientation.y,
        pos.orientation.z,
        pos.orientation.w)
#     euler = t.euler_from_quaternion(quaternion)
#     print(euler)
#     roll = euler[0] + 0.5
#     pitch = euler[1]  
#     yaw = euler[2] 
#     new_q = t.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
#     pos.orientation.x = new_q[0]
#     pos.orientation.y = new_q[1] 
#     pos.orientation.z = - new_q[2]
#     pos.orientation.w = new_q[3] 
#     R = t.rotation_matrix(0, [0, 0, 1], [0, 0, 0])
#     new_q = t.quaternion_from_matrix(R)
#     pos.orientation.x = new_q[0]
#     pos.orientation.y = new_q[1] 
#     pos.orientation.z = new_q[2]
#     pos.orientation.w = new_q[3] 
    changed.append( True)
    changed.append( True)
    changed.append( True)
    changed.append( False)
    changed.append( False)
    changed.append( False)    
    changed.append( False)
    publish_on_tf(last_position[:3], last_position[3:7], 'Kuka')
    return pos, changed, last_position

def publish_callback(data):
    pos = Pose()
    changed = []
    last_position = cur_pos[-1]
    pos.position.x = last_position[0]
    pos.position.y = last_position[1] 
    pos.position.z = last_position[2] 
    pos.orientation.x = last_position[3] 
    pos.orientation.y = last_position[4] 
    pos.orientation.z = last_position[5]
    pos.orientation.w = last_position[6] 
    publish_on_tf(last_position[:3], last_position[3:7], 'Kuka')

def vec_pos(pos):
    vec = []
    vec.append(pos.position.x)
    vec.append(pos.position.y)
    vec.append(pos.position.z)
    vec.append(pos.orientation.x)
    vec.append(pos.orientation.y)
    vec.append(pos.orientation.z)
    vec.append(pos.orientation.w)
    return vec

def pose_pos(vec):
    pos = Pose()
    pos.position.x = vec[0]
    pos.position.y = vec[1]
    pos.position.z = vec[2]
    pos.orientation.x = vec[3]
    pos.orientation.y = vec[4]
    pos.orientation.z = vec[5]
    pos.orientation.w = vec[6]
    return pos

def calc_next_pos(pos, changed, orig):
    dif = []
    new = []
    last_pos = cur_pos[-1]
    for i in range(0, len(last_pos)):
        if changed[i]:
                cur = last_pos[i]
                dif = pos[i] - cur
                new.append(cur + dif*step)
        elif i in (3,4,5,6):
                new.append(pos[i])
        else:
                new.append(orig[i])
    return(new)

def iterate_next_s_pos(X, count, orig):
    dif1 = []
    dif2 = []
    new = []
    last_pos = cur_pos[-1] 
    for i in range(0, len(last_pos)):
        # print(new, i)
        if i == 1:
            cur = last_pos[i]
            dif1 = X_s[0,count] - cur 
            new.append(cur + dif1*step)
        #     print("Y :", dif1, cur, cur + dif1*step)
        elif i == 2:  
            cur2 = last_pos[i]
            dif2 = X_s[1,count] - cur2 
            new.append(cur2 + dif2*step)
        #     print("Z : ", dif2, cur2, cur2 + dif2*step)
        #     print(count)
        else:
            new.append(orig[i]) 

    if (dif1<0.001 and dif2<0.001):
            print(new)   
            count=count+1      
    return(new, count)
    
    
def publish_on_tf(Position,  Rotation, name):
    br = tf.TransformBroadcaster()
    br.sendTransform((Position[0], Position[1], Position[2]), Rotation, rospy.Time.now(), name, "world_frame")       

def run_pos():
    vec = []
    rospy.Subscriber("/IIWA/Real_E_Pos", Pose, callback) 
#     print(cur_pos)
#     cur_pos.append([-0.11028514572397899, 0.0023238556311131796, 0.9595724234084408, 0.00268172342622, 0.86894326985, -0.000853314535667, -0.49490370174])
    rospy.sleep(1)
    target_position, changed, orig_position = target_pos()
    pub = rospy.Publisher("/IIWA/Desired_E_Pos", Pose, queue_size=3)
    print("cur_pos : ", pose_pos(cur_pos[-1]))
    print("Target position : ", target_position)
    vec = vec_pos(target_position)
    r = rospy.Rate(200)
    rospy.Subscriber("/IIWA/Real_E_Pos", Pose, publish_callback) 
    count = 0
    while not rospy.is_shutdown():
        # next_step = calc_next_pos(vec, changed, orig_position)
        next_step, count = iterate_next_s_pos(X_s, count, orig_position)
        # print(next_step)
        # publish_on_tf
        new_pos = pose_pos(next_step)
        pub.publish(new_pos)
        r.sleep()

 
#     r = rospy.Rate(100)
#     while True:
#         cur_pos.append(vec_pos(new_pos))
#         next_step = calc_next_pos(vec)
#         new_pos = pose_pos(next_step)
#         print(new_pos.position.z)
#         r.sleep()

main()




    
    