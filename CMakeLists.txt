cmake_minimum_required(VERSION 2.8.3)
project(v4l_tis)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nodelet
  roscpp
  sensor_msgs
  cmake_modules
)

FIND_PACKAGE(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES v4l_tis
#  CATKIN_DEPENDS cv_bridge image_transport nodelet roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ADD_LIBRARY(v4ldevice_tis SHARED src/v4ldevice.cpp)

LIST(APPEND LINK_LIBS
  v4ldevice_tis
 ${OpenCV_LIBS}
 ${catkin_LIBRARIES}
)

ADD_EXECUTABLE(view src/view.cpp)
target_link_libraries(view ${LINK_LIBS})
ADD_EXECUTABLE(listup src/listup.c)
add_executable(send src/send.cpp)
target_link_libraries(send ${LINK_LIBS})
