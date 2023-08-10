cmake_minimum_required(VERSION 2.8.3)
project(robot_setup_tf_tutorial)

find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs roscpp tf sti_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(odom_v7 src/odom_v7.cpp)
add_dependencies(odom_v7 ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_v7 ${catkin_LIBRARIES})

add_executable(odom_lidar src/odom_lidar.cpp)
add_dependencies(odom_lidar ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_lidar  ${catkin_LIBRARIES})


add_executable(convert_imuBno055 src/convert_imuBno055.cpp)
add_dependencies(convert_imuBno055 ${catkin_EXPORTED_TARGETS})
target_link_libraries(convert_imuBno055 ${catkin_LIBRARIES})

add_executable(odom_encoder src/odom_encoder.cpp)
add_dependencies(odom_encoder ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_encoder ${catkin_LIBRARIES})

# Install the executable
install(TARGETS odom_lidar odom_v7 convert_imuBno055 odom_encoder 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")