cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0003 NEW)
project(my_slam)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions pcl_ros image_transport cv_bridge sensor_msgs cmake_modules
  geometry_msgs visualization_msgs std_msgs tf message_generation message_filters rosbag rosconsole
)

set(CMAKE_BUILD_TYPE Debug)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES my_slam
  CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions pcl_ros roscpp sensor_msgs
  DEPENDS system_lib message_runtime std_msgs 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  include
)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_libraries(my_slam ${catkin_LIBRARIES})

####################################################
# Required 3rd party
####################################################
# PCL #######################
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# g2o #######################
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
link_libraries(${G2O_LIBRARIES})
message("G2o-libraries " ${G2O_LIBRARIES})

#CSPARSE
#find_package(CSPARSE REQUIRED)
#SET(CSPARSE_INCLUDE_DIR )


# Qt ########################
# http://qtnode.net/wiki/Qt4_with_cmake
find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})

# OpenMP ########################
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${QT_QTOPENGL_INCLUDE_DIR})

 	
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})


#SET(${CMAKE_LIBRARY_PATH} ${LIBRARY_OUTPUT_PATH})

ADD_DEFINITIONS(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)

#############################
# OpenCV ####################
#############################
#For using SIFT and SURF provide your own opencv installation
#via exporting a shell variable
set(OpenCV_DIR $ENV{OpenCV_DIR})
#or set it directly here, e.g.
#set(OpenCV_DIR /home/endres/ros_indigo/opencv2/build/)
#Then activate the following definition
#add_definitions(-DCV_NONFREE)

IF (NOT "${OpenCV_DIR}" EQUAL "")
    MESSAGE("Using OpenCV from " ${OpenCV_DIR})
ENDIF (NOT "${OpenCV_DIR}" EQUAL "")

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#############################
# Eigen ####################
#############################
find_package(Eigen)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# Suitesparse #######################
find_package(SuiteSparse REQUIRED)


##############################################################################
# Sources to Compile
##############################################################################
SET(ADDITIONAL_SOURCES src/main.cpp src/node.cpp src/matching_result.cpp src/openni_listener.cpp src/misc.cpp) #src/gicp-fallback.cpp src/main.cpp src/qtros.cpp  src/openni_listener.cpp src/qt_gui.cpp src/node.cpp src/glviewer.cpp src/parameter_server.cpp src/ros_service_ui.cpp src/misc.cpp src/landmark.cpp src/loop_closing.cpp src/ColorOctomapServer.cpp src/scoped_timer.cpp src/icp.cpp src/matching_result.cpp)
#GraphManager Files
SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/graph_manager.cpp) #src/graph_mgr_io.cpp src/graph_manager2.cpp src/graph_mgr_odom.cpp)
#SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/transformation_estimation_euclidean.cpp src/transformation_estimation.cpp src/graph_manager2.cpp src/header.cpp)
#Computation of meaningful covariances
#SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/covariance_estimation.cpp) # src/edge_labeler.cpp)
#Detector Threshold Adapter
SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/aorb.cpp) # src/feature_adjuster.cpp src/aorb.cpp src/features.cpp)

#IF (${USE_SIFT_GPU})
# 	SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/sift_gpu_wrapper.cpp)
#ENDIF (${USE_SIFT_GPU})

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS)

#include_directories(external/ /usr/include/suitesparse/)
include_directories(/usr/include/suitesparse/)
##############################################################################
# Binaries
##############################################################################
add_executable(my_slam ${ADDITIONAL_SOURCES}) #${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${ADDITIONAL_SOURCES})
#dd_dependencies(rgbdslam rgbdslam_gencpp)


SET(LIBS_LINK GL GLU cholmod cxsparse ${G2O_LIBRARIES} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS} ${CAMD_LIBRARY})

target_link_libraries(my_slam ${LIBS_LINK}  ${catkin_LIBRARIES} /opt/ros/indigo/lib/libg2o_interface.so /opt/ros/indigo/lib/libg2o_interactive.so)


## Declare a C++ library
# add_library(my_slam
#   src/${PROJECT_NAME}/my_slam.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(my_slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(my_slam_node src/my_slam_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(my_slam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(my_slam_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS my_slam my_slam_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
