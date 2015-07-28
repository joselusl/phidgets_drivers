cmake_minimum_required(VERSION 2.8.3)
project(phidgets_gps)

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

find_package(catkin REQUIRED
            COMPONENTS geometry_msgs nodelet phidgets_api roscpp sensor_msgs std_msgs std_srvs tf)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES phidgets_gps 
  CATKIN_DEPENDS geometry_msgs nodelet phidgets_api roscpp sensor_msgs std_msgs std_srvs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(phidgets_gps src/gps_ros_i.cpp include/phidgets_gps/gps_ros_i.h)
target_link_libraries(phidgets_gps ${catkin_LIBRARIES})
add_dependencies(phidgets_gps phidgets_api)

add_library(phidgets_gps_async_nodelet src/phidgets_gps_async_nodelet.cpp include/phidgets_gps/phidgets_gps_nodelet.h)
target_link_libraries(phidgets_gps_async_nodelet ${catkin_LIBRARIES} phidgets_gps)

add_library(phidgets_gps_sync_nodelet src/phidgets_gps_sync_nodelet.cpp include/phidgets_gps/phidgets_gps_nodelet.h)
target_link_libraries(phidgets_gps_sync_nodelet ${catkin_LIBRARIES} phidgets_gps)

add_executable(phidgets_gps_async_node src/phidgets_gps_async_node.cpp)
target_link_libraries(phidgets_gps_async_node ${catkin_LIBRARIES} phidgets_gps)

add_executable(phidgets_gps_sync_node src/phidgets_gps_sync_node.cpp)
target_link_libraries(phidgets_gps_sync_node ${catkin_LIBRARIES} phidgets_gps)


install(TARGETS phidgets_gps phidgets_gps_async_nodelet phidgets_gps_async_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS phidgets_gps phidgets_gps_sync_nodelet phidgets_gps_sync_node
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
