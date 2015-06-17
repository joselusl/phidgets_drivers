cmake_minimum_required(VERSION 2.8.3)
project(phidgets_gps)

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

find_package(catkin REQUIRED COMPONENTS geometry_msgs nodelet phidgets_api roscpp sensor_msgs std_msgs std_srvs tf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_gps 
  CATKIN_DEPENDS geometry_msgs nodelet phidgets_api roscpp sensor_msgs std_msgs std_srvs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_gps src/gps_ros_i.cpp)
add_library(phidgets_gps_nodelet src/phidgets_gps_nodelet.cpp)

add_executable(phidgets_gps_node src/phidgets_gps_node.cpp)

add_dependencies(phidgets_gps phidgets_api)

target_link_libraries(phidgets_gps ${catkin_LIBRARIES})
target_link_libraries(phidgets_gps_nodelet ${catkin_LIBRARIES} phidgets_gps)
target_link_libraries(phidgets_gps_node ${catkin_LIBRARIES} phidgets_gps)

install(TARGETS phidgets_gps phidgets_gps_nodelet phidgets_gps_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES phidgets_gps_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
