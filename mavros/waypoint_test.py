#! /home/ljz/anaconda3/envs/uav/bin/python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

import math
import rospy
import time
import serial
from threading import Thread
from std_msgs.msg import String
from rs_yolo.msg import Info
from geometry_msgs.msg import PoseStamped,Point,Twist
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import State,Altitude,PositionTarget

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
vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
point_pub = rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=10)

ser = serial.Serial()

def port_open_recv(path):#对串口的参数进行配置
    ser.port=path
    ser.baudrate=9600
    ser.bytesize=8
    ser.stopbits=1
    ser.parity="N"
    ser.open()
    if(ser.isOpen()):
        print(path+"串口打开成功！")
    else:
        print(path+"串口打开失败！")

def port_close():
    ser.close()
    if(ser.isOpen()):
        print("串口关闭失败！")
    else:
        print("串口关闭成功！")

def send(send_data):
    if(ser.isOpen()):
        ser.write(send_data.encode('utf-8'))#编码
        print('send {0} successfully!'.format(send_data))
        return True
    else:
        print('Fail to send {0}!'.format(send_data))
        return False

vel=Twist()
vel.linear.x=1
vel.linear.y=1
vel.linear.z=0.5

def send_pose(pose,speed=vel):
    vel_pub.publish(speed)
    pub.publish(pose)
    rate.sleep()

def reach_pose(x,y,z,err):
    if abs(x-pos_data.pose.position.x)>err or abs(y-pos_data.pose.position.y)>err or abs(y-pos_data.pose.position.y)>err:
        return False
    print('reach pose ({0},{1},{2})'.format(x,y,z))
    return True

def Quaternion_Euler_Transform(intput_data):
    """
        四元素转欧拉角
    """
    w = intput_data.w
    x = intput_data.x
    y = intput_data.y
    z = intput_data.z
 
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    '''
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    '''
    #rospy.loginfo("yaw=%.4f degree",yaw)
    return [roll,pitch,yaw]


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

    waypoint = PositionTarget()
    waypoint.position.x=0
    waypoint.position.y=0
    waypoint.position.z=0
    waypoint.velocity.x=0.5
    waypoint.velocity.y=0.5
    waypoint.velocity.z=1
    for i in range(30):
        if(not rospy.is_shutdown()):
            point_pub.publish(waypoint)
            rate.sleep()
        else:
            break

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True
    last_req = rospy.Time.now()
    
    #解算初始四元数和偏航角
    sum_yaw=0
    sum_alt=0
    print('Calculating orientation...')
    for i in range(10):
        sum_alt=sum_alt+pos_data.pose.position.z
        cur_yaw=Quaternion_Euler_Transform(pos_data.pose.orientation)[2]
        sum_yaw=sum_yaw+cur_yaw
        print('alt={0} m, yaw={1} rad = {2} degree'.format(pos_data.pose.position.z,cur_yaw,math.degrees(cur_yaw)))
        time.sleep(0.5)

    init_alt=sum_alt/10
    init_yaw=sum_yaw/10

    print('init_alt={0} m, init_yaw={1} rad'.format(init_alt,init_yaw))
    
    pose.pose.orientation=pos_data.pose.orientation
    waypoint.yaw=init_yaw

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

        waypoint.position.x=0
        waypoint.position.y=0
        waypoint.position.z=init_alt+2.5
        point_pub.publish(waypoint)
        rate.sleep()#防止解锁失败

    
    step=0
    drop=False
    drop_cnt=0
    reach_drop_point=False

    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        if step == 0:#起飞
            waypoint.position.x = 0
            waypoint.position.y = 0
            waypoint.position.z = init_alt+2.5
            point_pub.publish(waypoint)
            rate.sleep()
            #rospy.loginfo(pos_data.pose)
            if reach_pose(0,0,init_alt+2.5,0.1):
                for i in range(30):
                    send_pose(pose)
                    #rospy.loginfo(pos_data.pose)
                step=1

        elif step == 1:#投水点
            rot_xy=[4,0]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            waypoint.position.x = world_xy[0]
            waypoint.position.y = world_xy[1]
            waypoint.position.z = init_alt+2.5
            point_pub.publish(waypoint)
            rate.sleep()
            if reach_pose(world_xy[0],world_xy[1],init_alt+2.5,0.1):
               step = 2

        elif step == 2:#侦察点
            rot_xy=[4,4]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            waypoint.position.x = world_xy[0]
            waypoint.position.y = world_xy[1]
            waypoint.position.z = init_alt+2.5
            point_pub.publish(waypoint)
            rate.sleep()
            if reach_pose(world_xy[0],world_xy[1],init_alt+2.5,0.1):
               step = 3

        elif step == 3:#侦察点
            rot_xy=[0,4]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            waypoint.position.x = world_xy[0]
            waypoint.position.y = world_xy[1]
            waypoint.position.z = init_alt+2.5
            point_pub.publish(waypoint)
            rate.sleep()
            if reach_pose(world_xy[0],world_xy[1],init_alt+2.5,0.1):
               step = 4

        elif step == 4:#侦察点
            rot_xy=[0,0]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            waypoint.position.x = world_xy[0]
            waypoint.position.y = world_xy[1]
            waypoint.position.z = init_alt+2.5
            point_pub.publish(waypoint)
            rate.sleep()
            if reach_pose(world_xy[0],world_xy[1],init_alt+2.5,0.1):
               step = 5

        elif step == 5:
            offb_set_mode.custom_mode = 'AUTO.RTL'
            if (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                last_req=rospy.Time.now()
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("AUTO.RTL enabled")
                #rospy.loginfo(pos_data.pose)
                if abs(pos_data.pose.position.z) <= 0.1:
                    #rospy.loginfo("landed")
                    arm_cmd.value = False
                if not current_state.armed :
                    rospy.loginfo("Vehicle disarmed")
                rate.sleep()





