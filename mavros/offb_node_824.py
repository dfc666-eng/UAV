#! /home/ljz/anaconda3/envs/uav/bin/python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

import math
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

def Quaternion_Euler_Transform(intput_data):
    """
        四元素与欧拉角互换
    """

    w = intput_data.w
    x = intput_data.x
    y = intput_data.y
    z = intput_data.z
 
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    #print ('roll={0} rad, pitch={1} rad, yaw={2} rad'.format(roll,pitch,yaw))
    rospy.loginfo("yaw=%.4f degree",yaw)
    return [roll,pitch,yaw]

def Euler_Quaternion_Transform(intput_data):
    r = math.radians(intput_data[0]) #输入度
    p = math.radians(intput_data[1])
    y = math.radians(intput_data[2])
 
    sinp = math.sin(p/2)
    siny = math.sin(y/2)
    sinr = math.sin(r/2)
 
    cosp = math.cos(p/2)
    cosy = math.cos(y/2)
    cosr = math.cos(r/2)
 
    w = cosr*cosp*cosy + sinr*sinp*siny
    x = sinr*cosp*cosy - cosr*sinp*siny
    y = cosr*sinp*cosy + sinr*cosp*siny
    z = cosr*cosp*siny - sinr*sinp*cosy
    return [x,y,z,w]


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

    pose.pose.orientation.x=Euler_Quaternion_Transform([0,0,-90])[0]#逆时针为正
    pose.pose.orientation.y=Euler_Quaternion_Transform([0,0,-90])[1]
    pose.pose.orientation.z=Euler_Quaternion_Transform([0,0,-90])[2]
    pose.pose.orientation.w=Euler_Quaternion_Transform([0,0,-90])[3]
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
    init_orientation=pos_data.pose.orientation
    Quaternion_Euler_Transform(init_orientation)

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                
                last_req = rospy.Time.now()
        Quaternion_Euler_Transform(pos_data.pose.orientation)
        #rospy.loginfo(pos_data.pose.orientation)
        local_pos_pub.publish(pose)

        rate.sleep()


