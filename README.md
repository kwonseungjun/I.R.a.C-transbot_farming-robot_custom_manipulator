# I.R.a.C-transbot_farming-robot_custom_manipulator

### < Custom Mobile Robot System >

<img width="4032" height="3024" alt="IMG_2868" src="https://github.com/user-attachments/assets/2073bb60-1718-45fb-a023-49d519550ae0" />

#### This project is currently a work in progress.

This project is a ROS2-based mobile manipulation system built on the Transbot platform developed by YAHBOOM Robotics and the OpenManipulator platform developed by ROBOTIS.


### < Technical Challenges >

- Porting unsupported robotic systems to Ubuntu 22.04
  
- Real-time manipulation without MoveIt dependency

- Synchronization between mobile navigation and manipulator control
  
- Extending unsupported manipulator DOF configurations

### < Technical Hardware Summary >

The original OpenManipulator system was extensively modified and upgraded from a 4-DOF structure into a customized 6-DOF robotic manipulator to improve spatial manipulation capability and orientation control.


The system integrates multiple depth cameras with distinct roles: the Intel RealSense is utilized for perception via a YOLO-based object detection and segmentation pipeline, while the Astra Pro is dedicated to line tracing for autonomous mobile navigation.

To support higher computational requirements for real-time perception and manipulation, the embedded computing platform was upgraded from Jetson Nano to NVIDIA-based reComputer J401.


### < Key Features >

	⁃   ROS2-based robotic control architecture

	⁃	Customized 6-DOF robotic manipulator

	⁃	Real-time robotic perception using YOLO V8

	⁃	Multi-depth-camera integration

	⁃	Autonomous mobile manipulation system

	⁃	Embedded AI inference platform


### < Hardware >

	⁃	Transbot mobile platform (YAHBOOM Robotics)

	⁃	Customized OpenManipulator platform (ROBOTIS-based)

	⁃	Intel RealSense depth camera

	⁃	Astra Pro depth camera

	⁃	NVIDIA reComputer J401


### < Software >

	⁃	ROS2

	⁃	Ubuntu 22.04

	⁃	Python

	⁃	C++

	⁃	YOLOv8

	⁃	OpenCV

	⁃	TensorRT


### < Research Objectives >

	⁃	Autonomous mobile manipulation

	⁃	Real-time robotic perception

	⁃	Embedded robotic system integration

	⁃	Vision-based autonomous operation

	⁃	Robotic manipulation in unstructured environments


##

##### If you have any questions regarding this project, please contact us

###### 1. Mail adress: pharao7651@gmail.com
   
###### 2. Mail adress: anchan503@naver.com

###### 3. Mail adress: kwonsj75@icloud.com

#

###### Copyright ⓒ 2026 I.R.a.C Lab, K.I.T Univeersity.
###### Unauthorized copying or redistribution of this project is prohibited.

##### < Acknowledgements >
###### This project was developed based on the Transbot platform by YAHBOOM Robotics and the OpenManipulator platform by ROBOTIS. We sincerely appreciate their open robotic platforms and development resources.
