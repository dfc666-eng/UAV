#! /home/ljz/anaconda3/envs/uav/bin/python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

import math
import rospy
from geometry_msgs.msg import Twist
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
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    alt_sub = rospy.Subscriber('mavros/altitude', Altitude, altitude_cb)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    vel=Twist()
    vel.linear.x=0
    vel.linear.y=0
    vel.linear.z=0.2
    for i in range(100):
        if(not rospy.is_shutdown()):
            pub.publish(vel)
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

        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

        # 发布消息
        if alt_data.local>=2:
            vel.linear.z=0
        pub.publish(vel)
        rospy.loginfo(alt_data.local)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_position_target()
    except rospy.ROSInterruptException:
        pass


