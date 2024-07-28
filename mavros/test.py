#! /home/ljz/anaconda3/envs/uav/bin/python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import State,Altitude

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def altitude_cb(msg):
    global alt_data
    alt_data = msg

def send_position_target():
    # 初始化ROS节点
    rospy.init_node('test_node', anonymous=True)
    rate = rospy.Rate(20)  # 发布频率为20Hz

    # 创建发布者，发布到/mavros/setpoint_raw/local主题上
    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    alt_sub = rospy.Subscriber('mavros/altitude', Altitude, altitude_cb)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2
    for i in range(100):
        if(not rospy.is_shutdown()):
            pub.publish(pose)
            rate.sleep()
        else:
            break


    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    theta=0
    while not rospy.is_shutdown():
        # 创建一个PositionTarget消息对象
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # 设置期望的位置
        if theta>=6.28*2:
            offb_set_mode.custom_mode = 'AUTO.RTL'
            if(current_state.mode != "AUTO.RTL" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("AUTO.RTL")
            if(current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == False):
                    rospy.loginfo("Vehicle disarmed")
            rospy.loginfo(alt_data.local)
            continue

        pose.pose.position.x = 5*math.sin(theta)
        pose.pose.position.y = 5*math.cos(theta)
        pose.pose.position.z = 2.0
        theta=theta+0.02

        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

        # 发布消息
        pub.publish(pose)
        rospy.loginfo(alt_data.local)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_position_target()
    except rospy.ROSInterruptException:
        pass


