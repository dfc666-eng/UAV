#! /home/ljz/anaconda3/envs/uav/bin/python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""


import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
rc_data = ManualControl()

def state_cb(msg):
    global current_state
    current_state = msg

def pos_cb(msg):
    global pos_data
    pos_data = msg

def rc_cb(msg):
    global rc_data
    rc_data = msg

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped,pos_cb)
    rc_sub = rospy.Subscriber('mavros/manual_control/control',ManualControl,rc_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(100)

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
    offb_set_mode.custom_mode = 'STABILIZED'

    last_mode = 'STABILIZED'
    #arm_cmd = CommandBoolRequest()
    #arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if rospy.Time.now() - last_req > rospy.Duration(2.0):
            rospy.loginfo(current_state.mode)
            if current_state.mode != last_mode:#遥控切模式
                offb_set_mode.custom_mode = current_state.mode
            last_mode=current_state.mode#前一个时刻的模式
            last_req = rospy.Time.now()

        '''
        try:
            rospy.loginfo(pos_data.pose)
        except NameError:
            print('cannot get pose')
            pass
        try:
            rospy.loginfo(rc_data)
        except NameError:
            print('manual control lost')
            pass'''

        local_pos_pub.publish(pose)
        rate.sleep()


