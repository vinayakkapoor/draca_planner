cmake_minimum_required(VERSION 2.8.3)
project(draca_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  nodelet
  cmake_modules
  mrs_lib
  nav_msgs
  mrs_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
  multi_uav_dynreconfig
  swarm_control_manager
  swarm_utils
  message_generation
)


add_service_files(
  FILES
  ddrl_service_planning.srv
)



generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs  
  std_msgs
  sensor_msgs
  geometry_msgs
  mrs_msgs
)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${EIGEN3_LIBRARIES})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Draca_plannerController
  CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs nav_msgs message_runtime std_msgs
  DEPENDS Eigen
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(Draca_plannerController src/draca_planner_controller.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(Draca_plannerController
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )




catkin_install_python(PROGRAMS scripts/draca_planner_server.py
  DESTINATION $(CATKIN_PACKAGE_BINr_DESTINATION)
)


# install(TARGETS
# 		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
# 		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# 		FILES_MATCHING PATTERN "*.h"
# 		PATTERN ".svn" EXCLUDE
# )

# install(DIRECTORY doc
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
