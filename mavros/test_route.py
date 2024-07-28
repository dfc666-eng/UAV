#! /home/ljz/anaconda3/envs/uav/bin/python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""


import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def pos_cb(msg):
    global pos_data
    pos_data = msg

def reach_point(x,y,z,err):
    if abs(pos_data.pose.position.x-x)>err or abs(pos_data.pose.position.y-y)>err or abs(pos_data.pose.position.z-z)>err:
        return False
    return True

if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    local_pos_sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped,pos_cb)

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
    offb_set_mode.custom_mode = 'OFFBOARD'
    last_mode = 'OFFBOARD'
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    land_cnt=0 
    while(not rospy.is_shutdown()):
        if rospy.Time.now() - last_req > rospy.Duration(2.0):

            rospy.loginfo(current_state.mode)
            if current_state.mode != last_mode and current_state.mode != 'AUTO.LOITER':#遥控切模式
                offb_set_mode.custom_mode = current_state.mode
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    print('successfully switch to {0}'.format(current_state.mode))

            last_mode=current_state.mode#前一个时刻的模式

            if not current_state.armed:
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            try:
                rospy.loginfo(pos_data.pose)
            except NameError:
            #print('cannot get pose')
                pass
            last_req = rospy.Time.now()

        if land_cnt!=100:#未进入降落
            if reach_point(0,0,2,0.1):#到达指定点附近0.1范围内
                land_cnt=land_cnt+1            
                if land_cnt==100:
                    offb_set_mode.custom_mode = 'AUTO.LAND'
                print('ready to land: {0}%'.format(land_cnt))
            else:
                land_cnt=0
            local_pos_pub.publish(pose)
        rate.sleep()


