# rmep_base

## 文件结构

```
├── CMakeLists.txt
├── config
│   └── head_camera.yaml
├── launch
│   └── rmep_base.launch
├── LICENSE
├── msg
│   ├── GimbalCmd_ab.msg
│   ├── GimbalCmd.msg
│   └── GimbalFdb.msg
├── package.xml
├── readme.md
├── scripts
│   ├── rmep_base.py
│   └── test
│       └── rm_cam_test.py
└── srv
    ├── RobotArm.srv
    ├── RobotBlaster.srv
    ├── RobotGrip.srv
    └── RobotPwm.srv

```
## RMEP-SDK说明
https://robomaster-dev.readthedocs.io/zh_CN/latest/introduction.html

## 依赖

1. python3

## 安装robomaster python库

`pip install robomaster`

如果网络较差，多次都安装失败，可以尝试:

`pip install -i https://pypi.tuna.tsinghua.edu.cn/simple robomaster`

## 使用说明
1. 【注意】需要导入python robomaster库 pyhton3.6以上版本
2. 若使用摄像头需要修改rmep_base/config中的head_camera.yaml摄像头参数
2. 打开 rmep_base.launch 设置参数包括 
   连接模式 : ap (epwifi) 192.168.2.1
   	      sta (局域网) 192.168.42.2
   	      rndis (usb连接)
   底盘颜色、话题名字、模块是否存在等 
2. 运行命令 `roslaunch rmep_base rmep_base.launch`

## 发布话题（默认话题名字）
1. 图像数据（可选）
 /ep_cam/image_raw    图像话题
 /ep_cam/camera_info  相机参数文件
2. IMU数据
 /imu
3. 里程计数据
 /odom
4. TF数据（可选）
 /tf odom -> base_link
5. 云台姿态数据（可选）
 /ep_gimbal_fdb

## 订阅话题（默认话题名字）
1. 控制数据
 /cmd_vel
2. 云台控制数据
 /ep_gimbal_cmd

## 发布服务
1. 设置底盘输出六路PWM值
  /ep_pwm
2. 机械臂控制
  /ep_arm
3. /机械爪控制
  /ep_gripper

## 其他说明

launch文件的功能
- 启动EP-SDK功能包
- 可以在launch文件设置不同的配置
- 摄像头、云台、机械爪、机械臂


