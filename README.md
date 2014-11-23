ROS Package for Oculus Rift 
=============================
ROS driver package for Oculus Rift. 
See http://developer.oculusvr.com for Oculus Rift SDK

### Install
https://github.com/zchen24/OculusSDK
By downloading this SDK, you agree to Oculus's license. 
Tested on Ubuntu 12.04 LTS 

### Nodes


#### Publish
* /oculus/pose (geometry_msgs/PoseStamped)
* /oculus/md_info
* /tf

#### Params
* ~frequency
* ~parent_frame
* ~oculus_frame 

#### Subscribe
* left/image_raw (sensor_msgs/Image)
* right/image_raw (sensr_msgs/Image)

### License
TBD

