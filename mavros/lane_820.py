#! /home/ljz/anaconda3/envs/uav/bin/python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

import math
import rospy
import time
from threading import Thread
from std_msgs.msg import String
from rs_yolo.msg import Info
from geometry_msgs.msg import PoseStamped,Point
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import State, Altitude

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def altitude_cb(msg):
    global alt_data
    alt_data = msg

def pos_cb(msg):
    global pos_data
    pos_data = msg

def reach_pose(x,y,z,err):
    if abs(x-pos_data.pose.position.x)>err or abs(y-pos_data.pose.position.y)>err or abs(y-pos_data.pose.position.y)>err:
        return False
    return True

def doMsg(msg):
    global bucket_pos
    bucket_pos=msg
    #rospy.loginfo("bucket_pos=")
    #rospy.loginfo(msg)

# 初始化ROS节点
rospy.init_node('test_node', anonymous=True)

rate = rospy.Rate(20)  # 发布频率为20Hz

    # 创建发布者，发布到/mavros/setpoint_raw/local主题上
pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
alt_sub = rospy.Subscriber('mavros/altitude', Altitude, altitude_cb)
local_pos_sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped,pos_cb)
yolo_sub = rospy.Subscriber("detect_result_out", Info, doMsg, queue_size = 10)

def send_pose(pose):
    pub.publish(pose)
    rate.sleep()

if __name__ == '__main__':
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    for i in range(100):
        if(not rospy.is_shutdown()):
            send_pose(pose)
        else:
            break

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    last_req = rospy.Time.now()
   
    step=0
    init_alt=pos_data.pose.position.z
    while not rospy.is_shutdown():

        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()
        if (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            break
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = init_alt+4
        send_pose(pose)#防止解锁失败

    while not rospy.is_shutdown():
        
        pose.header.stamp = rospy.Time.now()
        if step == 0:
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = init_alt+4
            send_pose(pose)
            rospy.loginfo(pos_data.pose.position)
            if reach_pose(0,0,init_alt+4,err=0.1):
                for i in range(50):#悬停一会儿
                    send_pose(pose)
                    rospy.loginfo(pos_data.pose.position)
                step=1

        elif step == 1:
            pose.pose.position.x = 2
            pose.pose.position.y = 0
            pose.pose.position.z = init_alt+4
            send_pose(pose)
            rospy.loginfo(pos_data.pose.position)
            if reach_pose(2,0,init_alt+4,err=0.1):
                for i in range(50):
                    send_pose(pose)
                    rospy.loginfo(pos_data.pose.position)
                step=2
                
        elif step == 2:
            pose.pose.position.x = 2
            pose.pose.position.y = 2
            pose.pose.position.z = init_alt+4
            send_pose(pose)
            rospy.loginfo(pos_data.pose.position)
            if reach_pose(2,2,init_alt+4,err=0.1):
                for i in range(50):
                    send_pose(pose)
                    rospy.loginfo(pos_data.pose.position)
                step = 3

        elif step == 3:
            pose.pose.position.x = 0
            pose.pose.position.y = 2
            pose.pose.position.z = init_alt+4
            send_pose(pose)
            rospy.loginfo(pos_data.pose.position)
            if reach_pose(0,2,init_alt+4,err=0.1):
                for i in range(50):
                    send_pose(pose)
                    rospy.loginfo(pos_data.pose.position)
                step = 4

        elif step == 4:
            offb_set_mode.custom_mode = 'AUTO.RTL'
            #offb_set_mode.custom_mode = 'AUTO.PRECLAND'
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("AUTO.LAND enabled")
            rospy.loginfo(pos_data.pose.position)
            #rospy.loginfo("point: %f, %f, %f", bucket_pos.x, bucket_pos.y, bucket_pos.z)
            if abs(pos_data.pose.position.z) <= 0.1:
                rospy.loginfo("landed")
                arm_cmd.value = False
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                rospy.loginfo("Vehicle disarmed")
            rate.sleep()





