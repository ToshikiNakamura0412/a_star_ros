cmake_minimum_required(VERSION 3.8)
project(a_star_ros)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(obstacle_inflater_node src/obstacle_inflater.cpp)
target_link_libraries(obstacle_inflater_node ${catkin_LIBRARIES})

add_executable(a_star_node src/a_star.cpp)
target_link_libraries(a_star_node ${catkin_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
