cmake_minimum_required(VERSION 3.0.2)
project(cnc_control_v2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cnc1force_node src/CNC_1_Force.cpp)
add_executable(cnc2torque_node src/CNC_2_Torque.cpp)
add_executable(netft_force_sensor_publisher src/netft_force_sensor_publisher.cpp)

target_link_libraries(cnc1force_node ${catkin_LIBRARIES})
target_link_libraries(cnc2torque_node ${catkin_LIBRARIES})
target_link_libraries(netft_force_sensor_publisher ${catkin_LIBRARIES})

