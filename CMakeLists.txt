cmake_minimum_required(VERSION 3.0.2)
project(pose_cam_estimate)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  roslib
)

find_package(OpenCV REQUIRED)

find_package(PCL 1.14 REQUIRED)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  # ${PCL_LIBRARIES}
  # ${OpenCV_LIBS}
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pose_cam_estimate
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs
#  DEPENDS system_lib
)


include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_definitions(${PCL_DEFINITIONS})

add_definitions(${OpenCV_INCLUDE_DIRS})

# catkin_python_setup()

add_executable (lineSegmentation src/findLineSegment.cpp)

target_link_libraries (lineSegmentation ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})
# target_link_libraries (lineSegmentation ${catkin_LIBRARIES})

# add_executable (Extract_Border src/extractBorderFromRangedImage.cpp)

# target_link_libraries (Extract_Border ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_install_python(PROGRAMS 
                      src/get_corner_position_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)