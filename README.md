Setup
ROS2 Version: Galactic
JetPack Version: 5.1.5
L4T Version: 
Kernel Version: 
Ubuntu Version:

Install Steps (Need to Verify)
1. Install JetPack 5.1.5 using the NVIDIA SDK running on a Ubuntu laptop
2. Install ROS2 Galactic following the instructions online from ros2.org
3. Clone the contents into a directory called "controlbot_ws"
4. Install cv-bridge according to: https://index.ros.org/p/cv_bridge/#galactic
  NOTE: use the galactic branch before building: git clone https://github.com/ros-perception/vision_opencv.git -b galactic
5. Add serial port perms as per this stack overflow question: https://stackoverflow.com/questions/27858041/oserror-errno-13-permission-denied-dev-ttyacm0-using-pyserial-from-pyth
