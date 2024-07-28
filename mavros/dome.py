#! /usr/bin/env python
"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""



import rospy
import time
import math
from threading import Thread
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,Point,Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import State,Altitude
from rs_yolo.msg import Info


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

def doMsg(msg):
    rospy.loginfo("point: %f, %f, %f", msg.x, msg.y, msg.z)

def doMsg(msg):
    global bucket_pos
    bucket_pos=msg
    #rospy.loginfo("bucket_pos=")
    #rospy.loginfo(msg)

drop_cnt=0

state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
local_pos_sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped,pos_cb)
alt_sub = rospy.Subscriber('mavros/altitude', Altitude, altitude_cb)
local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
yolo_sub = rospy.Subscriber("detect_result_out", Info, doMsg, queue_size = 10)


if __name__ == "__main__":
    rospy.init_node("offb_node_py")


    

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    step=0
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        elif step != 4:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()
        
        local_pos_pub.publish(pose)
        pose.header.stamp = rospy.Time.now()
        rate.sleep()
        rospy.loginfo("point: %f, %f, %f, %s", bucket_pos.x, bucket_pos.y, bucket_pos.z,bucket_pos.classification)
        if step == 0:
            pose.pose.position.x = 10
            pose.pose.position.y = 0
            pose.pose.position.z = 2
            local_pos_pub.publish(pose)
            rate.sleep()
            if pos_data.pose.position.z >= 1.9 and pos_data.pose.position.z <= 2.1:
                for i in range(100):
                    local_pos_pub.publish(pose)
                    rate.sleep()
                step=1
        if step == 1:
            pose.pose.position.x = 10
            pose.pose.position.y = 10
            pose.pose.position.z = 2
            local_pos_pub.publish(pose)
            rate.sleep()
            if pos_data.pose.position.z >= 1.9 and pos_data.pose.position.z <= 2.1:
                for i in range(100):
                    local_pos_pub.publish(pose)
                    rate.sleep()
                try:
                        if bucket_pos.z!=0 and bucket_pos.classification!='none':
                            print('find bucket!') 
                            break
                except NameError:
                        pass
                if abs(bucket_pos.x-640)<=25 and abs(bucket_pos.y-360)<=25 and classification!='none':
                  rospy.loginfo("finish")
                  drop_cnt=drop_cnt+1
                else:
                  drop_cnt=0
                if drop_cnt == 20:
                  step=2
               
        if step == 2:
            pose.pose.position.x = 0
            pose.pose.position.y = 10
            pose.pose.position.z = 2
            local_pos_pub.publish(pose)
            rate.sleep()
            if pos_data.pose.position.z >= 1.9 and pos_data.pose.position.z <= 2.1:
                for i in range(100):
                    local_pos_pub.publish(pose)
                    rate.sleep()
                step=3
        if step == 3:
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 2
            local_pos_pub.publish(pose)
            rate.sleep()
            if pos_data.pose.position.z >= 1.9 and pos_data.pose.position.z <= 2.1:
                for i in range(100):
                    local_pos_pub.publish(pose)
                    rate.sleep()
                step=4

        if step == 4:
            offb_set_mode.custom_mode = 'AUTO.LAND'
            set_mode_client.call(offb_set_mode) 
            rospy.loginfo("AUTO.LAND enable")
            rospy.loginfo(step)
            if abs(pos_data.pose.position.z) <= 0.05:
                rospy.loginfo("landed")
                arm_cmd.value = False
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                rospy.loginfo("Vehicle disarmed")
            rate.sleep()
               
