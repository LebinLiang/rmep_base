#!/usr/bin/python3
# coding=UTF-8

'''
Filename: rmep_base.py
Author： Rm_camp
Time: 2022.1.10
Describe: EP API in ROS
'''

from robomaster import robot
from robomaster import camera
from robomaster import led
from robomaster import blaster
import cv2
import numpy as np
import rospy
import yaml
import time
import sys
import math
import string
import os
import threading
import rospkg
import tf
import copy

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rmep_msgs.srv import *
from rmep_msgs.msg import GimbalFdb,GimbalCmd

class EP_ROS:
    def __init__(self):
        self.connect_mode = str(rospy.get_param('connect_mode','ap')) #'ap' 'sta' 'rndis'  修改连接模式
        print("Robot Connect Mode: {0}".format(self.connect_mode))
        if self.connect_mode == 'ap': #奇怪的传参问题解决办法
            self.connect_mode = 'ap'
        elif self.connect_mode == 'sta':
            self.connect_mode = 'sta'
        elif self.connect_mode == 'rndis':
            self.connect_mode = 'rndis'

        self.resolution = rospy.get_param('/resolution','STREAM_360P') #STREAM_360P STREAM_720P 修改摄像头分辨率
        
        self.camera_image_topic = rospy.get_param('/camera_image_topic','ep_cam/image_raw') #图像话题名字
        self.camera_info_topic = rospy.get_param('/camera_info_topic','ep_cam/camera_info') #摄像头信息名字
        

        self.imu_topic = rospy.get_param('/imu_topic','imu')  #imu数据
        self.imu_frame_id = rospy.get_param('/imu_frame_id','imu_link') #imulink名字

        self.odom_topic = rospy.get_param('/odom_topic','odom') #odom话题名字
        self.odom_tf_switch = bool(rospy.get_param('/odom_tf_switch',True)) #odom tf变换 是否发布
        self.odom_frame_id = rospy.get_param('/odom_frame_id','base_link') #odom link名字
        
        self.cmd_topic = rospy.get_param('/cmd_topic','cmd_vel') #控制话题名字

        self.chassis_freq = int(rospy.get_param('~chassis_freq', 50)) #底盘数据更新频率
        self.imu_freq = rospy.get_param('/imu_freq',50) #imu数据更新频率
        self.gimbal_freq = rospy.get_param('/gimbal_freq',50) #云台数据更新频率

        self.is_arm = bool(rospy.get_param('/is_arm',False)) #False True 是否有机械臂
        self.is_gripper = bool(rospy.get_param('/is_gripper',False)) #False True 是否有机械爪
        self.is_camera = bool(rospy.get_param('/is_camera',False)) #False True 是否有摄像头
        self.is_gimbal = bool(rospy.get_param('/is_gimbal',False)) #False True 是否有云台

        print("Robot Arm : {0}".format(self.is_arm))
        print("Robot Grip : {0}".format(self.is_gripper))
        print("Robot Gimbal : {0}".format(self.is_gimbal))
        print("Robot Camera : {0}".format(self.is_camera))

        self.chassis_led_r = int(rospy.get_param('/led_r',255)) #red 0-255 LED颜色设置
        self.chassis_led_g = int(rospy.get_param('/led_g',0)) #green
        self.chassis_led_b = int(rospy.get_param('/led_b',0)) #blue

        self.chassis_shape_a = float(rospy.get_param('/chassis_shape_a',0.1)) #底盘尺寸 
        self.chassis_shape_b = float(rospy.get_param('/chassis_shape_b',0.1))
        self.chassis_wheel_r = float(rospy.get_param('/chassis_wheel_r',0.05))

        #话题发布
        self.odom_pub = rospy.Publisher(self.odom_topic,Odometry,queue_size=10) 

        if self.is_camera == True:
            self.image_pub = rospy.Publisher(self.camera_image_topic, Image, queue_size=10)
            self.camInfo_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=10, latch=True)
            ros_pack = rospkg.RosPack()
            self.camera_info_path = open(ros_pack.get_path('rmep_base') + '/config/head_camera.yaml') #摄像头标定信息yaml文件
        
        if self.is_gimbal == True:
            self.gimbal_pub = rospy.Publisher('ep_gimbal_fdb', GimbalFdb ,queue_size=10)
            self.gimbal_sub = rospy.Subscriber('ep_gimbal_cmd', GimbalCmd ,self.gimbal_cmd_cb,queue_size=1)
        
        self.imu_pub = rospy.Publisher(self.imu_topic,Imu, queue_size=10) 
        #控制话题订阅
        self.cmd_vel_sub = rospy.Subscriber(self.cmd_topic, Twist, self.cmd_vel_cb, queue_size=1) 
        #服务发布
        self.pwm_srv = rospy.Service('ep_pwm',RobotPwm ,self.pwm_srv_cb) 
 
        self.br = tf.TransformBroadcaster() 
        self.bridge = CvBridge()
        
        #底盘对象创建
        try:
            self.ep_robot = robot.Robot()
            self.ep_robot.initialize(conn_type=self.connect_mode)
            
            if self.is_camera == True:
                try:
                    self.ep_camera = self.ep_robot.camera
                    if self.resolution == 'STREAM_360P':
                        self.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
                    else:
                        self.ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
                except:
                    print("[ERROR]Please Check Your Robot Camera")
            
            self.ep_chassis = self.ep_robot.chassis
            self.ep_led = self.ep_robot.led
            self.ep_version = self.ep_robot.get_version()
            print("Robot Version: {0}".format(self.ep_version))
            if self.is_gimbal == True :
                try:
                    self.ep_gimbal = self.ep_robot.gimbal
                    self.ep_blaster = self.ep_robot.blaster
                    self.blaster_srv = rospy.Service('ep_blaster',RobotBlaster ,self.blaster_srv_cb)
                except:
                    print("[ERROR]Please Check Your Robot Gimbal")

            if self.is_arm == True :
                try:
                    self.ep_arm = self.ep_robot.robotic_arm
                    self.arm_srv = rospy.Service('ep_arm',RobotArm ,self.arm_srv_cb)

                except:
                    print("[ERROR]Please Check Your Robot Arm")
            
            if self.is_gripper == True :
                try:
                    self.ep_gripper = self.ep_robot.gripper
                    self.gripper_srv = rospy.Service('ep_gripper',RobotGrip ,self.gripper_srv_cb)

                except:
                    print("[ERROR]Please Check Your Robot Gripper")
        except:
            print("[WARNNING]Please Check Your Network")
            sys.exit()

        #变量设置
        self.chassis_position_x = 0
        self.chassis_position_y = 0
        self.chassis_position_yaw = 0
        self.chassis_velocity_x = 0
        self.chassis_velocity_y = 0
        self.chassis_velocity_wz = 0
        self.speed_w = [0,0,0,0]
        self.speed_bag_1 = []
        self.speed_bag_2 = []
        self.speed_bag_3 = []
        self.speed_bag_4 = []
        self.wz_bag = []
        self.original_x = 0
        self.original_y = 0
        self.chassis_position_x_tf = 0
        self.chassis_position_y_tf = 0
        self.is_original = True
        self.imu_yaw_theth = 0
        self.last_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.last_time_imu = rospy.Time.now()

        #设置订阅
        self.ep_led.set_led(comp=led.COMP_ALL, r=self.chassis_led_r, g=self.chassis_led_g, b=self.chassis_led_b, effect=led.EFFECT_ON)
        self.ep_chassis.sub_position(cs=1, freq=self.chassis_freq, callback=self.sub_position_handler)
        self.ep_chassis.sub_imu(freq=self.imu_freq, callback=self.sub_imu_info_handler)
        self.ep_chassis.sub_velocity(freq=self.chassis_freq, callback=self.sub_velocity_handler)
        self.ep_chassis.sub_esc(freq=self.chassis_freq, callback=self.sub_esc_info_handler)
        self.ep_chassis.sub_attitude(freq=self.chassis_freq, callback=self.sub_attitude_info_handler)
        if self.is_gimbal == True:
            self.ep_gimbal.sub_angle(freq=self.gimbal_freq, callback=self.sub_gimbal_handler)

        #设置图像发布线程
        if self.is_camera == True:
            self.make_camera_info()
            self.image_pub_thread = threading.Thread(target=self.image_pub_th)
            self.image_pub_thread.start()
        
        #设置底盘数据发布线程
        self.chassis_pub_thread = threading.Thread(target=self.chassis_pub_th)
        self.chassis_pub_thread.start()

    #发射器服务回调函数
    def blaster_srv_cb(self,blaster_req):
        if blaster_req.type == 0:
            self.ep_blaster.fire(fire_type=blaster.WATER_FIRE, times=blaster_req.count)
        else:
            self.ep_blaster.fire(fire_type=blaster.INFRARED_FIRE, times=blaster_req.count)
        return RobotBlasterResponse(1)

    #底盘pwm服务回调函数
    def pwm_srv_cb(self,pwm_req):
        self.ep_chassis.set_pwm_freq(pwm1= pwm_req.pwm1_freq, pwm2=pwm_req.pwm2_freq, pwm3=pwm_req.pwm3_freq, pwm4=pwm_req.pwm4_freq, pwm5=pwm_req.pwm5_freq, pwm6=pwm_req.pwm6_freq)
        self.ep_chassis.set_pwm_value(pwm1= pwm_req.pwm1_value, pwm2= pwm_req.pwm2_value, pwm3= pwm_req.pwm3_value, pwm4= pwm_req.pwm4_value, pwm5= pwm_req.pwm5_value, pwm6= pwm_req.pwm6_value) 
        return RobotPwmResponse(1)

    #机械臂服务回调函数
    def arm_srv_cb(self,arm_req):
        self.ep_arm.move(x=arm_req.x, y=arm_req.y).wait_for_completed()
        return RobotArmResponse(1)

    #机械爪服务回调函数
    def gripper_srv_cb(self,grip_req):
        if grip_req.state == 1:
            self.ep_gripper.open(power=50)
            rospy.sleep(grip_req.value)
            self.ep_gripper.pause()
            return RobotGripResponse(1)
        else:
            self.ep_gripper.close(power=100)
            rospy.sleep(grip_req.value)
            self.ep_gripper.pause()
            return RobotGripResponse(0)
    
    #订阅云台姿态数据 并发布ros消息
    def sub_gimbal_handler(self,angle_info):
        pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle = angle_info
        gimbal_info = GimbalFdb()
        gimbal_info.stamp = rospy.Time.now()
        gimbal_info.pitch_angle_fdb = pitch_angle
        gimbal_info.yaw_angle_fdb = yaw_angle
        gimbal_info.pitch_ground_angle_fdb = pitch_ground_angle
        gimbal_info.yaw_ground_angle_fdb = yaw_ground_angle
        self.gimbal_pub.publish(gimbal_info)
        #print("gimbal angle: pitch_angle:{0}, yaw_angle:{1}, pitch_ground_angle:{2}, yaw_ground_angle:{3}".format(pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle))
        
    #订阅底盘姿态数据
    def sub_attitude_info_handler(self,attitude_info):
        yaw, pitch, roll = attitude_info
        self.imu_yaw_theth = yaw / 57.3
        #print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(yaw, pitch, roll))

    #订阅底盘电调数据 
    '''
    【需要修改】TODO 修正里程计参数
    该函数订阅电调数据后进行数据均值滤波，正运动学解算获取里程计数据
    '''
    def sub_esc_info_handler(self,esc_info):
        speed, angle, timestamp, state = esc_info
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        #print("chassis esc: speed_w:{0}".format(speed))
        whel_k = [0.11,0.11,0.11,0.11]      # 轮子修正参数
        
        # 均值滤波与阈值
        for i in range(4):
            if abs(speed[i])<10:
                speed[i]=0

        self.speed_bag_1.append(copy.deepcopy(speed[0]))
        self.speed_bag_2.append(copy.deepcopy(speed[1]))
        self.speed_bag_3.append(copy.deepcopy(speed[2]))
        self.speed_bag_4.append(copy.deepcopy(speed[3]))

        if len(self.speed_bag_1)>10 :
            del self.speed_bag_1[0]
            del self.speed_bag_2[0]
            del self.speed_bag_3[0]
            del self.speed_bag_4[0]

        self.speed_w[0] = np.mean(self.speed_bag_1)
        self.speed_w[1] = np.mean(self.speed_bag_2)
        self.speed_w[2] = np.mean(self.speed_bag_3)
        self.speed_w[3] = np.mean(self.speed_bag_4)

        for i in range(4):
            if abs(self.speed_w[i])<10:
                self.speed_w[i]=0
            self.speed_w[i] = self.speed_w[i]*whel_k[i] 

        #print("chassis esc: speed_w:{0}".format(self.speed_w))
        a = self.chassis_shape_a
        b = self.chassis_shape_b
        r = self.chassis_wheel_r
        vx = -1*(-self.speed_w[0] + self.speed_w[1] + self.speed_w[2] - self.speed_w[3]) * r / 4
        vy = (-self.speed_w[0] - self.speed_w[1] + self.speed_w[2] + self.speed_w[3]) * r / 4 
        wz_raw = (-self.speed_w[0] - self.speed_w[1] - self.speed_w[2] - self.speed_w[3]) * r /math.sqrt(math.pow(a,2)+math.pow(b,2)) / 4
        
        if abs(wz_raw) < 0.3: 
            wz_raw = 0

        #print("chassis vx :{0}".format(vx))
        self.wz_bag.append(copy.deepcopy(wz_raw))
        if len(self.wz_bag)>10:
            del self.wz_bag[0]
        wz = np.mean(self.wz_bag) * 0.7
        
        if abs(wz) < 0.05:
            wz = 0
        #print("chassis vx :{0}".format(vx))
        self.chassis_velocity_x = vx
        self.chassis_velocity_y = vy
        self.chassis_velocity_wz = wz
        self.chassis_position_yaw += wz * dt

        if self.chassis_position_yaw > 3.14:
            self.chassis_position_yaw = self.chassis_position_yaw - 6.28
        if self.chassis_position_yaw < -3.14:
            self.chassis_position_yaw = self.chassis_position_yaw + 6.28

        self.chassis_position_x += (vx * math.cos(self.chassis_position_yaw) - vy * math.sin(self.chassis_position_yaw))* dt
        self.chassis_position_y += -1*((vx * math.sin(self.chassis_position_yaw) + vy * math.cos(self.chassis_position_yaw))* dt)
        self.last_time = self.current_time
        #print("chassis yaw :{0}".format(self.chassis_position_yaw*57.3)

    #订阅底盘速度信息
    '''
    订阅底盘速度信息 (暂时没有发布使用)
    '''
    def sub_velocity_handler(self,velocity_info):
        vgx, vgy, vgz, vbx, vby, vbz = velocity_info
        #print("chassis velocity: vbx:{0}, vby:{1}, vbz:{2}".format(vbx, vby, vbz))
        #self.chassis_velocity_x = vbx
        #self.chassis_velocity_y = vby
        # a = self.chassis_shape_a
        # b = self.chassis_shape_b
        # r = self.chassis_wheel_r
        # pi = math.pi
        # w0 = (vbx - vby + 0 * (a + b)) * (30 / pi) / r
        # w1 = (vbx + vby - 0 * (a + b)) * (30 / pi) / r
        # w2 = (vbx - vby - 0 * (a + b)) * (30 / pi) / r
        # w3 = (vbx + vby + 0 * (a + b)) * (30 / pi) / r
        # print("chassis w: w1:{0}, w2:{1}, w3:{2}, w4:{3}".format(w0, w1, w2 , w3))
        # c1 = self.speed_w[0]/ w0
        # c2 = self.speed_w[1]/ w1
        # c3 = self.speed_w[2]/ w2
        # c4 = self.speed_w[3]/ w3
        #print("chassis c: c1:{0}, c2:{1}, c3:{2}, c4:{3}".format(c1, c2, c3 , c4))

    
    #订阅底盘IMU数据
    '''
    获取imu数据并发布imu话题
    '''
    def sub_imu_info_handler(self,imu_info):
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
        #print("chassis imu: acc_x:{0}, acc_y:{1}, acc_z:{2}, gyro_x:{3}, gyro_y:{4}, gyro_z:{5}".format(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z))
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = self.imu_frame_id
        imu_msg.angular_velocity.x = gyro_x
        imu_msg.angular_velocity.y = gyro_y
        imu_msg.angular_velocity.z = gyro_z
        imu_msg.linear_acceleration.x = acc_x
        imu_msg.linear_acceleration.y = acc_y
        imu_msg.linear_acceleration.z = acc_z
        quat = tf.transformations.quaternion_from_euler(0, -0, self.imu_yaw_theth)
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]
        self.imu_pub.publish(imu_msg)

    #订阅底盘位置信息
    '''
    获取节点开启后为原点的位置信息
    '''
    def sub_position_handler(self,position_info):
        x, y, yaw = position_info
        if self.is_original == True:
            self.original_x = x 
            self.original_y = y 
            self.is_original = False
        self.chassis_position_x_tf  = x - self.original_x
        self.chassis_position_y_tf = y - self.original_y 
        #print("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, yaw))

     # Camera_info 数据包
    def make_camera_info(self):
        self.camera_info = CameraInfo()
        self.cam_data = yaml.safe_load(self.camera_info_path )

        self.camera_info.header = rospy.Header(stamp = rospy.Time.now())
        self.camera_info.header.frame_id = self.cam_data["camera_name"]

        self.camera_info.distortion_model = self.cam_data["distortion_model"]
        self.camera_info.width = self.cam_data['image_width']
        self.camera_info.height = self.cam_data['image_height']
        self.camera_info.binning_x = 0
        self.camera_info.binning_y = 0

        self.camera_info.K = self.cam_data['camera_matrix']['data']
        self.camera_info.D = self.cam_data['distortion_coefficients']['data']
        self.camera_info.R = self.cam_data['rectification_matrix']['data']
        self.camera_info.P = self.cam_data['projection_matrix']['data']
        self.camInfo_pub.publish(self.camera_info)

    #图像发布线程
    def image_pub_th(self):
        while not rospy.is_shutdown():
            #try:
                img = self.ep_camera.read_cv2_image()
                header = Header(stamp = rospy.Time.now())
                header.frame_id = self.camera_info.header.frame_id
                cv_img = self.bridge.cv2_to_imgmsg(img, "bgr8")
                size = img.shape
                cv_img.header=header
                cv_img.width = size[1]
                cv_img.height = size[0]
                cv_img.encoding = "bgr8"
                cv_img.data = cv_img.data
                self.camera_info.header = header
                self.camInfo_pub.publish(self.camera_info)
                self.image_pub.publish(cv_img) #发布消息
                cv2.waitKey(1)
            # except:
            #     print("[ERROR]Please Check Your Robot Camera12")
            #     sys.exit()
    
    #底盘数据发布线程
    def chassis_pub_th(self):
        chassis_rate = rospy.Rate(self.chassis_freq)
        while not rospy.is_shutdown():
            odom_msg =  Odometry()
            odom_msg.header.frame_id = "odom"
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.pose.pose.position.x = self.chassis_position_x
            odom_msg.pose.pose.position.y = self.chassis_position_y
            odom_msg.pose.pose.position.z = 0

            quat = tf.transformations.quaternion_from_euler(0,0,-self.chassis_position_yaw)
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            
            odom_msg.child_frame_id = "base_link"
            odom_msg.twist.twist.linear.x = self.chassis_velocity_x
            odom_msg.twist.twist.linear.y = self.chassis_velocity_y
            odom_msg.twist.twist.angular.z = self.chassis_velocity_wz

            if  self.odom_tf_switch == True:
                self.br.sendTransform((self.chassis_position_x_tf, -self.chassis_position_y_tf, 0),
                              tf.transformations.quaternion_from_euler(0, -0, -self.imu_yaw_theth),
                              rospy.Time.now(),
                              "/base_link",
                              "/odom")

            self.odom_pub.publish(odom_msg)
            chassis_rate.sleep()
    
    #订阅云台控制话题
    '''
    云台运动控制
    '''
    def gimbal_cmd_cb(self,msg):
        self.ep_gimbal.moveto(pitch=msg.pitch, yaw=msg.yaw, pitch_speed=msg.pitch_speed, yaw_speed=msg.yaw_speed).wait_for_completed()

    #订阅控制话题
    '''
    底盘运动控制
    '''
    def cmd_vel_cb(self, msg):
        self.move_with_wheel_speed(msg.linear.x, -msg.linear.y, -msg.angular.z * 57.29578) #
        #self.move_with_chassis_control(msg.linear.x, msg.linear.y, -msg.angular.z * 57.29578) #

    #运动控制命令（控制底盘）
    def move_with_chassis_control(self, vx=0.0, vy=0.0 ,wz = 0.0):
        #print("chassis control: vx:{0}, vy:{1}, wz:{2}".format(vx, vy, wz))
        self.ep_chassis.drive_speed(vx, vy ,wz)
    
    #运动控制命令（控制电机）
    def move_with_wheel_speed(self, x=0.0, y=0.0, yaw=0.0):
        yaw = -yaw / 57.3
        a = self.chassis_shape_a
        b = self.chassis_shape_b
        r = self.chassis_wheel_r
        pi = math.pi
        w0 = (x - y + yaw * (a + b)) * (30 / pi) / r
        w1 = (x + y - yaw * (a + b)) * (30 / pi) / r
        w2 = (x - y - yaw * (a + b)) * (30 / pi) / r
        w3 = (x + y + yaw * (a + b)) * (30 / pi) / r
        self.ep_chassis.drive_wheels(w0, w1, w2, w3)

    #退出EP控制函数
    def ep_exit(self):
        self.ep_led.set_led(comp=led.COMP_ALL, r=255, g=255, b=255, effect=led.EFFECT_ON)
        if self.is_camera == True:
            self.ep_camera.stop_video_stream()
        self.ep_chassis.unsub_position()
        self.ep_chassis.unsub_imu()
        self.ep_chassis.unsub_velocity()
        self.ep_chassis.unsub_esc()
        self.ep_chassis.unsub_attitude()
        if self.is_gimbal == True:
            self.ep_gimbal.unsub_angle()
        self.ep_robot.close()
        print("[INFO]Close EP")

if __name__ == '__main__':
    
    rospy.init_node('rmep_base', anonymous=True)
    my_ep_robot = EP_ROS()
    rospy.spin()
    my_ep_robot.ep_exit()
    
    # try:
        
    # except:
    #     print("[INFO]Exit")
    
    
    
        