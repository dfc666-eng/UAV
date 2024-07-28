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
from mavros_msgs.msg import State,Altitude

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
#vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

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

def send_pose(pose):
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
    for i in range(30):
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
    
    #解算初始四元数和偏航角
    sum_yaw=0
    sum_alt=0
    print('Calculating orientation...')
    init_alt=pos_data.pose.position.z
    init_yaw=Quaternion_Euler_Transform(pos_data.pose.orientation)[2]
    print('init_alt={0} m, init_yaw= {1} rad = {2} degree'.format(init_alt,init_yaw,math.degrees(init_yaw)))
    
    pose.pose.orientation=pos_data.pose.orientation

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
        pose.pose.position.z = init_alt+2.5
        send_pose(pose)#防止解锁失败

    
    step=0
    drop=False
    drop_cnt=0
    reach_drop_point=True


    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        if step == 0:#起飞
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = init_alt+2.5
            send_pose(pose)
            #rospy.loginfo(pos_data.pose.position)
            if reach_pose(0,0,init_alt+2.5,0.2):
                for i in range(30):
                    send_pose(pose)
                    #rospy.loginfo(pos_data.pose.position)
                step=1

        elif step == 1:#前往投水点
            i=0
            while i<=30.5:
                rot_xy=[i,0]
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+2.5
                send_pose(pose)
                #rospy.loginfo(pos_data.pose.position)
                i=i+0.1
            step = 2
        elif step == 2:#找桶
            rospy.loginfo("reach pose (%.3f, %.3f, %.3f)",pos_data.pose.position.x,pos_data.pose.position.y,pos_data.pose.position.z)
            i=0
            while i<=3.1415:#圆心(30,0) 6*2椭圆
                xx=30+0.5*math.cos(i)
                yy=-3*math.sin(i)
                rot_xy=[xx,yy]
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+2.5
                send_pose(pose)
                if pos_data.pose.position.z<=init_alt+1.8:
                    send('S')
                    step = 3
                    break
                try:
                    if bucket_pos.z!=0 and bucket_pos.classification!='none':
                        print('find bucket!')
                        step=3
                        break
                except NameError:
                    pass
                i=i+0.02
            step = 3
        elif step == 3:#投水点
            if not reach_drop_point:#到达投水点打开串口
                reach_drop_point=True
                try:
                    port_open_recv('/dev/ttyUSB1')
                except serial.SerialException:
                    print('cannot open ttyUSB1!')
                    pass

            elif reach_drop_point:
                init_time = rospy.Time.now()
                while not drop:
                    if rospy.Time.now()-init_time>rospy.Duration(15.0):
                        print('timeout!')
                        send('S')
                        break
                    pose.pose.position.z = init_alt+2.5
                        #rospy.loginfo(pos_data.pose)
                    try:
                        if (rospy.Time.now() - last_req) > rospy.Duration(3.0):#每隔一段时间进行一次校准
                            rospy.loginfo("point: %f, %f, %f, %s", bucket_pos.x, bucket_pos.y, bucket_pos.z,bucket_pos.classification)
                            rot_xy=[0.2+bucket_pos.y,bucket_pos.x]#对准桶
                            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
                            pose.pose.position.x = pos_data.pose.position.x+world_xy[0]
                            pose.pose.position.y = pos_data.pose.position.y+world_xy[1]
                            last_req = rospy.Time.now()
                            if bucket_pos.classification!='none':
                                print('try to move to the bucket')
                            else:
                                print('fail to detect bucket')

                        send_pose(pose)

                        if abs(bucket_pos.x)<=0.05 and abs(bucket_pos.y)<=0.05 and bucket_pos.z!=0:#桶位于中心，且不是误识别为000
                            drop_cnt=drop_cnt+1
                            print('detected:{0}%'.format(drop_cnt*4))
                        else:
                            drop_cnt=0
                        if drop_cnt == 25:#投水
                            print('drop water successfully!')
                            send('S')
                            drop = True

                    except NameError:
                        pose.pose.position.x = pos_data.pose.position.x
                        pose.pose.position.y = pos_data.pose.position.y
                        send_pose(pose)
                        pass
                    
                    #rospy.loginfo(pos_data.pose)
                #投水完成，悬停一会儿
                pose.pose.position.x = pos_data.pose.position.x
                pose.pose.position.y = pos_data.pose.position.y
                pose.pose.position.z = init_alt+2.5
                for i in range(30):
                    send_pose(pose)
                    #rospy.loginfo(pos_data.pose)
                port_close()
                step=4
        elif step == 4:#侦察点
            rot_xy=[51,0]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            pose.pose.position.x = world_xy[0]
            pose.pose.position.y = world_xy[1]
            pose.pose.position.z = init_alt+2.5
            send_pose(pose)
            if reach_pose(world_xy[0],world_xy[1],init_alt+2.5,0.1):
                print('start detecting!')
                for i in range(50):
                    send_pose(pose)
                step=5

        elif step == 5:
            rot_xy=[51,0]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            pose.pose.position.x = world_xy[0]
            pose.pose.position.y = world_xy[1]
            pose.pose.position.z = init_alt+2.5
            send_pose(pose)
            if reach_pose(world_xy[0],world_xy[1],init_alt+2.5,0.1):
                for i in range(50):
                    send_pose(pose)
                step=6         


        elif step == 6:
            rot_xy=[51,3]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            pose.pose.position.x = world_xy[0]
            pose.pose.position.y = world_xy[1]
            pose.pose.position.z = init_alt+2.5
            send_pose(pose)
            if reach_pose(world_xy[0],world_xy[1],init_alt+2.5,0.1):
                for i in range(50):
                    send_pose(pose)
                step=7            


        elif step == 7:
            rot_xy=[55,3]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            pose.pose.position.x = world_xy[0]
            pose.pose.position.y = world_xy[1]
            pose.pose.position.z = init_alt+2.5
            send_pose(pose)
            if reach_pose(world_xy[0],world_xy[1],init_alt+2.5,0.1):
                for i in range(50):
                    send_pose(pose)
                step=8            


        elif step == 8:
            rot_xy=[55,-3]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            pose.pose.position.x = world_xy[0]
            pose.pose.position.y = world_xy[1]
            pose.pose.position.z = init_alt+2.5
            send_pose(pose)
            if reach_pose(world_xy[0],world_xy[1],init_alt+2.5,0.1):
                for i in range(50):
                    send_pose(pose)
                step=9            

        elif step == 9:
            rot_xy=[51,-3]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            pose.pose.position.x = world_xy[0]
            pose.pose.position.y = world_xy[1]
            pose.pose.position.z = init_alt+2.5
            send_pose(pose)
            if reach_pose(world_xy[0],world_xy[1],init_alt+2.5,0.1):
                for i in range(50):
                    send_pose(pose)
                step=10           

        elif step == 10:
            rot_xy=[51,0]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            pose.pose.position.x = world_xy[0]
            pose.pose.position.y = world_xy[1]
            pose.pose.position.z = init_alt+6
            send_pose(pose)
            if reach_pose(world_xy[0],world_xy[1],init_alt+6,0.1):
                for i in range(50):
                    send_pose(pose)
                print('return!')
                step=11            

        elif step == 11:
            rot_xy=[0,0]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
            pose.pose.position.x = world_xy[0]
            pose.pose.position.y = world_xy[1]
            pose.pose.position.z = init_alt+6
            send_pose(pose)
            if reach_pose(world_xy[0],world_xy[1],init_alt+6,0.1):
                for i in range(50):
                    send_pose(pose)
                    #rospy.loginfo(pos_data.pose)
                step = 12

        elif step == 12:
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





