cmake_minimum_required(VERSION 2.8.3)
project(sti_module)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  # fiducial_msgs
  apriltag_ros
  std_msgs
  roscpp
  rospy 
  tf
  tf2_ros
  dynamic_reconfigure
  sti_msgs
)

catkin_package(

)

add_service_files(FILES AddTwoInts.srv)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(getPoseRobot_v4 src/getPoseRobot_v4.cpp)
add_dependencies(getPoseRobot_v4 ${catkin_EXPORTED_TARGETS})
target_link_libraries(getPoseRobot_v4 ${catkin_LIBRARIES})

add_executable(getPoseRobot_v6 src/getPoseRobot_v6.cpp)
add_dependencies(getPoseRobot_v6 ${catkin_EXPORTED_TARGETS})
target_link_libraries(getPoseRobot_v6 ${catkin_LIBRARIES})

add_executable(getPoseAruco_v2 src/getPoseAruco_v2.cpp)
add_dependencies(getPoseAruco_v2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(getPoseAruco_v2 ${catkin_LIBRARIES})


add_executable(key_pub src/key_pub.cpp)
add_dependencies(key_pub ${catkin_EXPORTED_TARGETS})
target_link_libraries(key_pub ${catkin_LIBRARIES})

add_executable(ghi_LOG src/ghi_LOG.cpp)
add_dependencies(ghi_LOG ${catkin_EXPORTED_TARGETS})
target_link_libraries(ghi_LOG ${catkin_LIBRARIES})

install(TARGETS getPoseRobot_v4 
                getPoseAruco_v2 
                key_pub ghi_LOG 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data)

install(PROGRAMS src/quytrinh.py 
                 src/aruco_plan.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

