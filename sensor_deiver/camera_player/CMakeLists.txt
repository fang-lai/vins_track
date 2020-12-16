cmake_minimum_required(VERSION 2.8.3)
project(camera_player)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS "-std=c++11")
# set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
# set (CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
roscpp rospy sensor_msgs std_msgs  tf cv_bridge   image_geometry  dynamic_reconfigure
)
generate_dynamic_reconfigure_options(config/config.cfg) 

find_package(OpenCV 3.0 QUIET)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)
add_library(cap_lib
src/V4L2Capture.cpp
)

catkin_package(
  CATKIN_DEPENDS 
)



add_executable(play_opencv_node   src/play_opencv_node.cpp )

add_dependencies(play_opencv_node ${PROJECT_NAME}_gencfg)
target_link_libraries(play_opencv_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS} 
 )


 add_executable(play_v4l2_node    src/play_v4l2_node.cpp )
 target_link_libraries(play_v4l2_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS} 
   cap_lib
 )
 
 add_executable(play_topic_node   src/play_topic_node.cpp )
 target_link_libraries(play_topic_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS} 
 )

#############
## Install ##
#############


install(DIRECTORY config include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(TARGETS
  cap_lib
  play_opencv_node
  play_v4l2_node
  play_topic_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

