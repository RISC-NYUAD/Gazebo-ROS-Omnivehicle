cmake_minimum_required(VERSION 2.8.3)
project(omni_vehicle)
add_compile_options(-std=c++11)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} /usr/include/eigen3)
link_directories(${GAZEBO_LIBRARY_DIRS} /usr/lib/x86_64-linux-gnu)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(omni_teleoper_node src/my_teleoper_node.cpp)
target_link_libraries(omni_teleoper_node ${catkin_LIBRARIES} )

add_library(omni_controller_plugin SHARED src/controller.cpp)
target_link_libraries(omni_controller_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

