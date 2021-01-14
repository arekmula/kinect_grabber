# kinect_grabber
ROS opencv example

## Installation: 
- install OpenCV dependencies:
    ```
    sudo apt-get install ros-kinetic-cv-bridge
    sudo apt-get install ros-kinetic-cv-camera
    ```
- install ros Kinect drivers:
    ```
    sudo apt install ros-melodic-freenect-launch
    ```
  
- clone repository:
	```
  cd ~/catkin_ws/src
  git clone https://github.com/dominikbelter/kinect_grabber
  ```
  
- compile example:
    ```
    cd ~/catkin_ws
    catkin_make
    ```
  
## To run:
- set camera (0 - first camera):

  `rosparam set cv_camera/device_id 0 `
  
- run:  
    ```
    cd ~/catkin_ws
    roslaunch freenect_launch freenect.launch
    rosrun kinect_grabber kinect_grabber_node
    ```
