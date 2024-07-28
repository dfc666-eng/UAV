#! /home/ljz/anaconda3/envs/uav/bin/python3

"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""

#初始化起飞位置，低速矩形侦察航线，回中后低速返回
#可修改矩形边长，侦察速度，返航速度
#2min5s

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

ser = serial.Serial()#串口

detect_rate=0.04#侦察速度，两航点的间隔
home_rate=0.2#返航速度0.2
setoff_rate=0.1#出发速度0.1

bucket_dist=31#起飞点到投水区的距离31
detect_dist=51#起飞点到侦察区的距离51

cam_dist=0.16#相机到水中心的距离
rect_length=6#矩形长6
rect_width=3#矩形宽3

rt_alt=4#返航高度5.5
cruise_alt=2#巡航高度2.5
detect_delay=20#每个侦察航点停留时间（循环次数）

quick_return=False#不开启快速返航
search_bucket=False#向右找桶

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
    if abs(x-pos_data.pose.position.x)>err or abs(y-pos_data.pose.position.y)>err or abs(z-pos_data.pose.position.z)>err:
        return False
    #print('reach pose ({0},{1},{2})'.format(x,y,z))
    rospy.loginfo('reach pose (%.3f, %.3f, %.3f)',x,y,z)
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

def Euler_Quaternion_Transform(input_data):
    #欧拉角转四元数，输入弧度
    r = input_data[0] 
    p = input_data[1]
    y = input_data[2]
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
    
    #打开串口
    try:
        port_open_recv('/dev/ttyUSB0')
    except serial.SerialException:
        print('cannot open ttyUSB0!')
        pass

    #解算初始四元数和偏航角
    sum_yaw=0
    sum_alt=0
    sum_x=0
    sum_y=0
    print('Calculating initial orientation and position...')
    for i in range(10):
        sum_x=sum_x+pos_data.pose.position.x
        sum_y=sum_y+pos_data.pose.position.y
        sum_alt=sum_alt+pos_data.pose.position.z
        cur_yaw=Quaternion_Euler_Transform(pos_data.pose.orientation)[2]
        sum_yaw=sum_yaw+cur_yaw
        #print('alt={0} m, yaw={1} rad = {2} degree'.format(pos_data.pose.position.z,cur_yaw,math.degrees(cur_yaw)))
        print('processing: {0}%'.format((i+1)*10))
        time.sleep(0.2)

    init_x=sum_x/10
    init_y=sum_y/10
    init_alt=sum_alt/10
    init_yaw=sum_yaw/10
    init_alt=pos_data.pose.position.z
    init_yaw=Quaternion_Euler_Transform(pos_data.pose.orientation)[2]

    #init_yaw=0
    #print('init_alt={0} m, init_yaw= {1} rad = {2} degree'.format(init_alt,init_yaw,math.degrees(init_yaw)))
    rospy.loginfo("\ninit_alt=%.3fm\ninit_yaw=%.3frad=%.3fdegree",init_alt,init_yaw,math.degrees(init_yaw))
    rospy.loginfo("\ninit_x=%.3fm\ninit_y=%.3fm",init_x,init_y)
 
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

        pose.pose.position.x = init_x+0
        pose.pose.position.y = init_y+0
        pose.pose.position.z = init_alt+cruise_alt
        send_pose(pose)#防止解锁失败

    
    step=0
    drop=False
    drop_cnt=0
    reach_drop_point=False

    pose.pose.orientation.x=Euler_Quaternion_Transform([0,0,init_yaw])[0]
    pose.pose.orientation.y=Euler_Quaternion_Transform([0,0,init_yaw])[1]
    pose.pose.orientation.z=Euler_Quaternion_Transform([0,0,init_yaw])[2]
    pose.pose.orientation.w=Euler_Quaternion_Transform([0,0,init_yaw])[3] 

    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        if step == 0:#起飞
            pose.pose.position.x = init_x+0
            pose.pose.position.y = init_y+0
            pose.pose.position.z = init_alt+cruise_alt
            send_pose(pose)
            #rospy.loginfo(pos_data.pose.position)
            if reach_pose(init_x+0,init_y+0,init_alt+cruise_alt,0.3):
                for i in range(30):
                    send_pose(pose)
                    #rospy.loginfo(pos_data.pose.position)
                step=1

        elif step == 1:#前往投水点
            i=0
            while i<=bucket_dist:
                rot_xy=[i,0]
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+cruise_alt
                send_pose(pose)
                i=i+setoff_rate
                try:
                    if i>=bucket_dist-1 and bucket_pos.z!=0 and bucket_pos.classification!='none':
                        print('find bucket!')
                        step=2
                        break
                except NameError:
                    pass
            while not reach_pose(world_xy[0],world_xy[1],init_alt+cruise_alt,0.2):
                send_pose(pose)
            
            temp_x=i#找到桶时的x
            if search_bucket and step == 1:#向右平移3.5m寻找桶
                print('look for bucket...')
                i=0
                while i<=3.5:
                    rot_xy=[temp_x,-i]
                    world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                    pose.pose.position.x = world_xy[0]
                    pose.pose.position.y = world_xy[1]
                    pose.pose.position.z = init_alt+cruise_alt
                    send_pose(pose)
                    i=i+setoff_rate
                    try:
                        if bucket_pos.z!=0 and bucket_pos.classification!='none':
                            print('find bucket!')
                            break
                    except NameError:
                        pass
                while not reach_pose(world_xy[0],world_xy[1],init_alt+cruise_alt,0.2):
                    send_pose(pose)
            step = 2

        elif step == 2:#投水点
            if not reach_drop_point:#到达投水点打开串口
                reach_drop_point=True

            elif reach_drop_point:
                
                init_time = rospy.Time.now()
                while not drop:
                    if rospy.Time.now()-init_time>rospy.Duration(12.0):
                        print('timeout!')
                        send('S')
                        break
                    pose.pose.position.z = init_alt+cruise_alt
                        #rospy.loginfo(pos_data.pose)
                    try:
                        if (rospy.Time.now() - last_req) > rospy.Duration(3.0):#每隔一段时间进行一次校准
                            rospy.loginfo("point: %f, %f, %f, %s", bucket_pos.x, bucket_pos.y, bucket_pos.z,bucket_pos.classification)
                            rot_xy=[cam_dist+bucket_pos.y,bucket_pos.x]#对准桶
                            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw), rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)]
                            pose.pose.position.x = pos_data.pose.position.x+world_xy[0]
                            pose.pose.position.y = pos_data.pose.position.y+world_xy[1]
                            last_req = rospy.Time.now()
                            if bucket_pos.classification!='none':
                                print('try to move to the bucket')
                            else:
                                print('fail to detect bucket')

                        send_pose(pose)

                        if abs(bucket_pos.x)<=0.05 and abs(bucket_pos.y+cam_dist)<=0.05 and bucket_pos.z!=0:#桶位于(0,-cam_dist)，且不是误识别为000
                            drop_cnt=drop_cnt+1
                            print('detected:{0}%'.format(drop_cnt*5))
                        else:
                            drop_cnt=0
                        if drop_cnt == 20:#投水
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
                pose.pose.position.z = init_alt+cruise_alt
                for i in range(50):
                    send_pose(pose)
                    #rospy.loginfo(pos_data.pose)
                port_close()
                step=4

        elif step == 4:#侦察点
            rot_xy=[detect_dist,0]
            world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
            pose.pose.position.x = world_xy[0]
            pose.pose.position.y = world_xy[1]
            pose.pose.position.z = init_alt+cruise_alt
            send_pose(pose)
            if reach_pose(world_xy[0],world_xy[1],init_alt+cruise_alt,0.2):
                print('start detecting!')
                for i in range(50):
                    send_pose(pose)
                step=5
     
        elif step == 5:
            i=0
            while i<=rect_length/2:
                rot_xy=[detect_dist,i]#(51,3)
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+cruise_alt
                send_pose(pose)
                i=i+detect_rate
            while not reach_pose(world_xy[0],world_xy[1],init_alt+cruise_alt,0.2):
                send_pose(pose)
            for i in range(detect_delay):
                send_pose(pose)
            step=6            


        elif step == 6:
            i=0
            while i<=rect_width:
                rot_xy=[detect_dist+i,rect_length/2]#(55,3)
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+cruise_alt
                send_pose(pose)
                i=i+detect_rate
            while not reach_pose(world_xy[0],world_xy[1],init_alt+cruise_alt,0.2):
                send_pose(pose)
            for i in range(detect_delay):
                send_pose(pose)
            step=7            


        elif step == 7:
            i=0
            while i<=rect_length:
                rot_xy=[detect_dist+rect_width,rect_length/2-i]#(55,-3)
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+cruise_alt
                send_pose(pose)
                i=i+detect_rate
            while not reach_pose(world_xy[0],world_xy[1],init_alt+cruise_alt,0.2):
                send_pose(pose)
            for i in range(detect_delay):
                send_pose(pose)
            step=8            

        elif step == 8:
            i=0
            while i<=rect_width:
                rot_xy=[detect_dist+rect_width-i,-rect_length/2]#(51,-3)
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+cruise_alt
                send_pose(pose)
                i=i+detect_rate
            while not reach_pose(world_xy[0],world_xy[1],init_alt+cruise_alt,0.2):
                send_pose(pose)
            for i in range(detect_delay):
                send_pose(pose)
            step=9           

        elif step == 9:
            i=0
            while i<=rect_length/2:
                rot_xy=[detect_dist,-rect_length/2+i]#(51,0)
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+cruise_alt
                send_pose(pose)
                i=i+detect_rate
            while not reach_pose(world_xy[0],world_xy[1],init_alt+cruise_alt,0.2):
                send_pose(pose)
            for i in range(detect_delay):
                send_pose(pose)
            step=10   

        elif step == 10:#矩形中心
            i=0
            while i<=rect_width/2:
                rot_xy=[detect_dist+i,0]#(53,0,2.5)
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+cruise_alt
                send_pose(pose)
                i=i+detect_rate
            while not reach_pose(world_xy[0],world_xy[1],init_alt+cruise_alt,0.2):
                send_pose(pose)
            for i in range(100):
                send_pose(pose)
            step=11    

        elif step == 11:#升高
            i=0
            while i<=rt_alt-cruise_alt:
                rot_xy=[detect_dist+rect_width/2,0]#(53,0,5.5)
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+cruise_alt+i
                send_pose(pose)
                i=i+detect_rate
            while not reach_pose(world_xy[0],world_xy[1],init_alt+rt_alt,0.2):
                send_pose(pose)
            print('return!')
            step=12   
        
        elif step == 12:#返航
            if quick_return:
                rot_xy=[0,0]
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+init_alt+rt_alt
                send_pose(pose)
                if reach_pose(world_xy[0],world_xy[1],init_alt+rt_alt,0.2):
                    for i in range(20):
                        send_pose(pose)
                    step = 13
                continue

            i=0
            while i<=detect_dist+rect_width/2:
                rot_xy=[detect_dist+rect_width/2-i,0]
                world_xy=[rot_xy[0]*math.cos(init_yaw)-rot_xy[1]*math.sin(init_yaw)+init_x, rot_xy[0]*math.sin(init_yaw)+rot_xy[1]*math.cos(init_yaw)+init_y]
                pose.pose.position.x = world_xy[0]
                pose.pose.position.y = world_xy[1]
                pose.pose.position.z = init_alt+rt_alt
                send_pose(pose)
                i=i+home_rate
            while not reach_pose(world_xy[0],world_xy[1],init_alt+rt_alt,0.2):
                send_pose(pose)
            step=13

        elif step == 13:
            offb_set_mode.custom_mode = 'AUTO.RTL'
            if (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                last_req=rospy.Time.now()
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("AUTO.RTL enabled")
                #rospy.loginfo(pos_data.pose)
                if abs(pos_data.pose.position.z) <= init_alt+0.1:
                    #rospy.loginfo("landed")
                    arm_cmd.value = False
                if not current_state.armed :
                    rospy.loginfo("Vehicle disarmed")
                rate.sleep()





