cmake_minimum_required(VERSION 2.8.3)
project(move_group)
SET(CMAKE_CXX_FLAGS "-std=c++11")
##set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  roscpp
  rospy
  actionlib
  rosconsole
  grasp_planning_msgs 
  move_base_agv_msgs
  message_generation
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_sensor_msgs
  cv_bridge
  tf
  tf2
  pcl_conversions
  pcl_ros
  rosconsole
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

set(OpenCV_FOUND 1)
SET(OpenCV_DIR /home/weili/opencv/build)
find_package(OpenCV REQUIRED )

set(OPENCV_INCLUDES /home/weili/opencv)
include_directories(${OPENCV_INCLUDES}/include)

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
 add_message_files(
   FILES
   move_base.msg
   ready.msg
 )
## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
## add_action_files(
##   FILES
##   MoveBaseAGV.action
## )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   nav_msgs
 )
# We should try to get Qt's include flags automatically somehow
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
find_package(Qt4Core)
ADD_DEFINITIONS(${QT_DEFINITIONS})
# Move_group include files
set(MOVE_GROUP_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/../move_group)
include_directories(${MOVE_GROUP_INCLUDES}/include)

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
  LIBRARIES move_group
  CATKIN_DEPENDS moveit_ros_planning_interface roscpp
  DEPENDS system_lib   
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CML_INCLUDES /home/weili/simple_kinova/src/turtle/include)
include_directories(
 ${CML_INCLUDES}
 ${CML_INCLUDES}/linuxcan/include
 ${CML_INCLUDES}/xcommunication
 ${CML_INCLUDES}/xsens
  /usr/local/xsens/include
  /usr/include
)


link_directories(
  /usr/local/xsens/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)
## Declare a C++ library
# add_library(move_group
#   src/${PROJECT_NAME}/move_group.cpp
# )
SET(CML 
    /home/weili/simple_kinova/src/turtle/CML/Amp.cpp
    /home/weili/simple_kinova/src/turtle/CML/AmpFile.cpp
    #CML/AmpFW.cpp
    /home/weili/simple_kinova/src/turtle/CML/AmpParam.cpp
    /home/weili/simple_kinova/src/turtle/CML/AmpPDO.cpp
    /home/weili/simple_kinova/src/turtle/CML/AmpPVT.cpp
    /home/weili/simple_kinova/src/turtle/CML/AmpStruct.cpp
    /home/weili/simple_kinova/src/turtle/CML/AmpUnits.cpp
    /home/weili/simple_kinova/src/turtle/CML/AmpVersion.cpp
    /home/weili/simple_kinova/src/turtle/CML/Can.cpp
    /home/weili/simple_kinova/src/turtle/CML/can_copley.cpp
    /home/weili/simple_kinova/src/turtle/CML/can_kvaser.cpp
    #CML/can_ixxat.cpp
    #CML/can_ixxat_v3.cpp
    /home/weili/simple_kinova/src/turtle/CML/CanOpen.cpp
    /home/weili/simple_kinova/src/turtle/CML/CML.cpp
    /home/weili/simple_kinova/src/turtle/CML/CopleyIO.cpp
    /home/weili/simple_kinova/src/turtle/CML/CopleyIOFile.cpp
    /home/weili/simple_kinova/src/turtle/CML/CopleyNode.cpp
    /home/weili/simple_kinova/src/turtle/CML/ecatdc.cpp
    /home/weili/simple_kinova/src/turtle/CML/Error.cpp
    /home/weili/simple_kinova/src/turtle/CML/EtherCAT.cpp


    /home/weili/simple_kinova/src/turtle/CML/EventMap.cpp
    /home/weili/simple_kinova/src/turtle/CML/File.cpp
    /home/weili/simple_kinova/src/turtle/CML/Filter.cpp
    #CML/Firmware.cpp
    /home/weili/simple_kinova/src/turtle/CML/Geometry.cpp
    /home/weili/simple_kinova/src/turtle/CML/InputShaper.cpp
    /home/weili/simple_kinova/src/turtle/CML/IOmodule.cpp

    /home/weili/simple_kinova/src/turtle/CML/Linkage.cpp
    /home/weili/simple_kinova/src/turtle/CML/LSS.cpp
    /home/weili/simple_kinova/src/turtle/CML/Network.cpp
    /home/weili/simple_kinova/src/turtle/CML/Node.cpp

    /home/weili/simple_kinova/src/turtle/CML/Path.cpp
    /home/weili/simple_kinova/src/turtle/CML/PDO.cpp
    /home/weili/simple_kinova/src/turtle/CML/Reference.cpp
    /home/weili/simple_kinova/src/turtle/CML/SDO.cpp
    /home/weili/simple_kinova/src/turtle/CML/Threads.cpp
    /home/weili/simple_kinova/src/turtle/CML/Threads_posix.cpp
    /home/weili/simple_kinova/src/turtle/CML/TrjScurve.cpp
    /home/weili/simple_kinova/src/turtle/CML/Utils.cpp
)

SET(XSENS_PUBLIC
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/enumerateusbdevices.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/int_xsdatapacket.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/iointerface.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/iointerfacefile.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/legacydatapacket.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/mtwsdidata.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/legacydatapacket.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/packetfixeddata.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/packetstamper.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/protocolhandler.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/serialinterface.cpp
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/synclinegmt.c
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/synclinemk4.c
    /home/weili/simple_kinova/src/turtleXsens/xcommunication/usbinterface.cpp
    )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(move_group ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
 add_executable(move_group_node src/main.cpp src/CombineTree.cpp src/CombineTree2.cpp src/BackHandTree.cpp src/BackTreeNode.cpp src/StartBaseTreeNode.cpp src/BaseTrajectory.cpp src/Commen.cpp src/StartBaseTree.cpp src/StartTree.cpp src/StartTreeNode.cpp src/GraspMobilePosePlan.cpp  ${CML})

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(move_group_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(move_group_node
   ${catkin_LIBRARIES}
/home/weili/linuxcan/canlib/libcanlib.so
 )

## Declare a C++ executable
 add_executable(pub_point_cloud_collision_node src/PubPointCloudCollision.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(pub_point_cloud_collision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(pub_point_cloud_collision_node
   ${catkin_LIBRARIES}
 )


## Declare a C++ executable
## add_executable(order_arm_node src/OrderArm.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
## add_dependencies(order_arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
## target_link_libraries(order_arm_node
##   ${catkin_LIBRARIES}
## )

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
# install(TARGETS move_group move_group_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_move_group.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
