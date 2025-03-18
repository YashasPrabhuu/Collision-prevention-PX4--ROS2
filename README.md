# Collision-prevention-PX4-ROS2
Implementation of ROS2 on PX4 platform to detect Obstacles and avoid them using a 2d-lidar.
## Hardware used
- LD-06 lidar
- RPI-4
- Cube pilot -orange
- Holybro X500 V2

## Parameters on QGC that needs to be changed
- Change the following params on the QGC to establish communication between the Cube and companion computer
  ```bash
    MAV_1_CONFIG = TELEM2
    UXRCE_DDS_CFG = 0 (Disabled)
    SER_TEL2_BAUD = 57600
  #Following paramaters are to enable collision prevention
  CP_DIST =
  CP_DELAY =
  CP_GUIDE_ANG =
  CP_GO_NO_DATA =
  MPC_POS_MODE =

## Procedure and setup of companion computer
### Wiring setup 
- Refer:https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html
### Folder structure and Software setup
- The following steps show how to install and set up Ubuntu 22.04 on the RPi-4
- Install Ubuntu 22.04 onto the RPi:
- Open a new terminal and install raspi-config:
  ```bash
    sudo apt update
    sudo apt upgrade
    sudo apt-get install raspi-config

 - Open raspi-config:
    ```bash
    sudo raspi-config

  - Go to the **Interface** Option and then click **Serial Port**.

  - Select **No** to disable the serial login shell.
  - Select **Yes** to enable the serial interface.
  - Click **Finish** and restart the RPi.

- Open the firmware boot configuration file in the nano editor on RPi:
  ```bash
  sudo nano /boot/firmware/config.txt

- Append the following text to the end of the file (after the last line):
  ```bash
  enable_uart=1
  dtoverlay=disable-bt

- Then save the file and restart the RPi.

- Check that the serial port is available. In this case we use the following terminal commands to list the serial devices:

  ```bash
  cd /
  ls /dev/ttyAMA0

- The result of the command should include the RX/TX connection /dev/ttyAMA0 (note that this serial port is also available as /dev/serial0).
The RPi is now setup to work with RPi and communicate using the /dev/ttyAMA0 serial port. Note that we'll install more software in the following sections to work with MAVLink and ROS 2.

- Open a new terminal and follow the instructions to set up the file structure.  
- Install and set up ROS2 humble base version.
- The following folder structure is important to avoid dependency issues and should be created in the home directory.
- Setting up Micro-XRCE-DDS-Agent
  ```bash
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  cd Micro-XRCE-DDS-Agent
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig /usr/local/lib/

  
- Check whether the client side of the uxrce DDS is turned on by typing the following command into the MAVlink console on QGC.If the MAVlink console responds with "service already in process, " your client is running successfully. (Ref: Companion computer setup PX4 document)
  ```bash
  uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600
  uxrce_dds_client status
  
- On the RPi terminal enter the following command to start the Uxrce agent
  ```bash
  
  sudo MicroXRCEAgent serial --dev /dev/serial0 -b 921600

- Now that the agent and client are running, you should see activity on the MAVLink console and the RPi terminal. You can view the available topics using the following command on the RPi:
  ```bash
  source /opt/ros/humble/setup.bash
  ros2 topic list

- Create your ROS2 workspace to execute the Collision prevention node(Python), this step applies to both on the Rpi and on the PC.
  ```bash
  mkdir -p ros2_ws/src
  cd ros2_ws/src
  ros2 pkg create PX4_collision_prevention --build-type ament_python
  cd PX4_collision_prevention
  #Under your package cd to your_package_nam and git clone the following repo.
  git clone https://github.com/PX4/px4_msgs.git
  git clone https://github.com/YashasPrabhuu/Collision-prevention-PX4-ROS2.git
  #head to root of your current directory 
  cd ..
  source /opt/ros/humble/setup.bash
  colcon build

- Cloning LD-06 lidar ROS2 driver.
   ```bash
  cd root
  git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
  cd ldlidar_ros2_ws
  source install/setup.bash
  #Run the following command to launch the node to get the scan data on the topic named"/scan"
  ros2 launch ldlidar_stl_ros2 ld06.launch.py
  #If the node launches wothout any error,then the data is published on the scan topic.
  #Check the following by running the following command in an another terminal
  ros2 topic list | grep /scan
  #To see the distance and intensities msgs run the following command
  ros2 topic echo /scan
 

- Open a new terminal and source the environment.
  ```bash
  cd ros2_ws
  source install/local_setup.bash
  #To launch your node
  ros2 run px4_lidarmsg_publisher lidardata 

## Simulation setup for obstacle detection using 2D lidar.
- OS: ubuntu 22.04 
- Install and set up ROS2 humble.
- The following folder structure is important to avoid dependency issues.
- Setting up PX4 folder
  ```bash
  mkdir Collision_prevention
  cd Collision_prevention
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
  cd PX4-Autopilot/
  make px4_sitl
- Open a new terminal and change to directory PX4 and execute the following command.
  ```bash
  make px4_sitl gz_x500_lidar_2d

![Screenshot from 2025-01-03 15-02-04](https://github.com/user-attachments/assets/ca7a81c3-8a99-4c86-9cb0-5b00440a3b5d)

- Install MicroXRCE agent to communicate between PX4 and companion computer/ Laptop, change the directory to the Micro XRCE directory and execute the following command.
  ```bash
  cd Collision_prevention
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
  cd Micro-XRCE-DDS-Agent
  mkdir build
  cd build
  cmake ..
  make
  sudo make install
  sudo ldconfig /usr/local/lib/
  #After the above step if you are not in the root of Micro-XRCE-DDS-Agent folder the run the following command.
  
  cd Micro-XRCE-DDS-Agent
  #STart the agent
  MicroXRCEAgent udp4 -p 8888

- Open Q-ground control and check that the connection between Gazebo sim and the QGC is established.
This can be confirmed when topics appear on the Micro xrce terminal or one can "ros2 topic list"
to check out the topics.
- Enable joystick input on QGC to control the drone.
- To visualize the detected obstacles enable the obstacle overlay on the map.This can be done by navigating to "Vehicle setup" > "Safety" > "Obstacle detection" > enable "show obstacle overlay".     


## git repositories and documents referred.
- ROS2 integration PX4 document
  - https://docs.px4.io/main/en/ros2/user_guide.html
- Companion computer setup PX4 document.
  - https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html
- LD-06 lidar ROS2 setup it repo
  - https://github.com/ldrobotSensorTeam/ldlidar_stl_ros?tab=readme-ov-file#Instructions
- Collision prevention setup and companion computer param setup
  - https://docs.px4.io/main/en/computer_vision/collision_prevention.html 
