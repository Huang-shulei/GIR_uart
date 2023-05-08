cmake_minimum_required(VERSION 3.10.2)
project(GIR_uart)


# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  serial
  std_msgs
  sensor_msgs
  dynamic_reconfigure
)

catkin_package()

include_directories(
 include 
 ${catkin_INCLUDE_DIRS})



#可执行文件 串口节点
add_executable(uart_node src/uart_node.cpp )
target_link_libraries( uart_node  ${catkin_LIBRARIES})
#链接库文件 协议编码/解码
add_library(protocol SHARED src/protocol.cpp)
target_link_libraries( protocol  ${catkin_LIBRARIES})
target_link_libraries( uart_node  protocol)

