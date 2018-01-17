cmake_minimum_required(VERSION 2.8.3)
project(kinect_anywhere)

set(CMAKE_CXX_FLAGS "-std=c++11  ${CMAKE_CXX_FLAGS}")
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake CACHE INTERNAL "" FORCE)

find_package(ZeroMQ)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
  camera_info_manager
  message_generation
)

find_package(PCL REQUIRED)

add_message_files(FILES
  BodyFrame.msg
  JointPositionAndState.msg
  Body.msg
)

generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  std_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ZeroMQ_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(body_frame src/body_frame.cpp)
target_link_libraries(body_frame ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES})

add_executable(color_frame src/color_frame.cpp)
target_link_libraries(color_frame ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES})

add_executable(point_cloud src/point_cloud.cpp)
target_link_libraries(point_cloud ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES})
