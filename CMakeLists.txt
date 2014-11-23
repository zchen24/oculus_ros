cmake_minimum_required(VERSION 2.8.3)
project(oculus_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  tf
)

# Oculus SDK
FIND_PACKAGE(OculusSDK REQUIRED)
include_directories(${OculusSDK_INCLUDE_DIRS})
link_directories(${OculusSDK_LIBRARY_DIRS})

# OpenCV
find_package( OpenCV REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oculus_ros
  CATKIN_DEPENDS roscpp tf geometry_msgs
  DEPENDS OculusSDK
)

###########
## Build ##
###########

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# sensor
add_executable(oculus_node
  src/oculus_rift.h
  src/oculus_rift.cpp
  src/oculus_node.cpp)

target_link_libraries(oculus_node
  ${catkin_LIBRARIES}
  ${OculusSDK_LIBRARIES})


# viewer
add_executable(oculus_viewer_node
  src/oculus_shader.h
  src/oculus_shader.cpp
  src/oculus_viewer_node.cpp
  shader/oculus.v.glsl
  shader/oculus.f.glsl)

target_link_libraries(oculus_viewer_node
  ${catkin_LIBRARIES}
  ${OculusSDK_LIBRARIES}
  ${OpenCV_LIBS}
  glut GLEW)




