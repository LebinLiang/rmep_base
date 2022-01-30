#!/usr/bin/python3
# coding=UTF-8

import cv2
from robomaster import robot
from robomaster import camera
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
import time
from sensor_msgs.msg import CameraInfo
import yaml

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    rospy.init_node('ep_cam', anonymous=True) #定义节点
    image_pub = rospy.Publisher('/ep_cam/image_raw', Image, queue_size = 10) #定义话题
    camInfo_pub = rospy.Publisher("/s1_camInfo", CameraInfo, queue_size=10)
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    cam_file = open('/home/michael/rm_ws/src/camera_info/head_camera.yaml')
    cam_data = yaml.safe_load(cam_file)
    bridge = CvBridge()
    ep_chassis = ep_robot.chassis
    


    while not rospy.is_shutdown():    # Ctrl C正常退出，如果异常退出会报错device busy！
        #start = time.time()
        img = ep_camera.read_cv2_image()
        #frame = cv2.flip(frame,0)   #垂直镜像操作
        #img = cv2.flip(img,1)   #水平镜像操作   
        cv2.imshow("Robot", img)
        #ros_frame = Image()
        #img = cv2.resize(img_raw,(0,0),360,640,interpolation=cv2.INTER_NEAREST)
        #v2.imshow("Robot2", img)
        header = Header(stamp = rospy.Time.now())
        header.frame_id = cam_data["camera_name"]
        # ros_frame.header=header
        # ros_frame.width = cam_data['image_width']
        # ros_frame.height = cam_data['image_height']
        # ros_frame.encoding = "bgr8"
        cv_img = bridge.cv2_to_imgmsg(img, "bgr8")
        size = img.shape
        #print(cv_img)
        cv_img.header=header
        cv_img.width = size[1]
        cv_img.height = size[0]
        cv_img.encoding = "bgr8"
        #cv_img.step = size[1]*3
        cv_img.data = cv_img.data
        #ros_frame.data = np.array(img).tostring() #图片格式转换
        image_pub.publish(cv_img) #发布消息
        #end = time.time()  
        #print("cost time:", end-start ) # 看一下每一帧的执行时间，从而确定合适的rate
        #rate = rospy.Rate(25) # 10hz 
        cv2.waitKey(1)

    ep_camera.stop_video_stream()
    ep_robot.close()
    cv2.destroyAllWindows() 
    print("quit successfully!")

  
    # for i in range(0, 200):
    #     img = ep_camera.read_cv2_image()
    #     cv2.imshow("Robot", img)
    #     cv2.waitKey(1)
    # cv2.destroyAllWindows()
    # ep_camera.stop_video_stream()

    # ep_robot.close()
