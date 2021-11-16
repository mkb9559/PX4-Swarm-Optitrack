cmake_minimum_required(VERSION 2.8)
project(px4_zxz)
#set(OpenCV_DIR /usr/local/opencv341/share/OpenCV)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
set(CMAKE_CXX_STANDARD 11)

find_package (catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS geometry_msgs roscpp
    DEPENDS Boost
)
include_directories(${catkin_INCLUDE_DIRS})




add_executable(single_offboard
    src/single/single_offboard.cpp
)
target_link_libraries(single_offboard
    ${catkin_LIBRARIES}
    boost_system
    boost_filesystem
)




add_executable(swarm_cmd
    src/swarm/swarm_cmd.cpp
)
target_link_libraries(swarm_cmd
    ${catkin_LIBRARIES}
    boost_system
    boost_filesystem
)





install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

