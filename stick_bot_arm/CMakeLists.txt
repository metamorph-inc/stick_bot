cmake_minimum_required(VERSION 2.8.3)
project(stick_bot_arm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hebi_cpp_api
  joy
  joy_teleop
  roscpp
  hebiros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES 
    stick_bot_arm
  CATKIN_DEPENDS 
    geometry_msgs 
    hebi_cpp_api 
    joy 
    joy_teleop 
    roscpp
  DEPENDS
    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

add_executable(hebi_turret_node
  src/hebi_turret.cpp)
target_link_libraries(hebi_turret_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(hebi_arm_node
  src/hebi_arm.cpp)
target_link_libraries(hebi_arm_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(hebi_gripper_node
  src/hebi_gripper.cpp)
target_link_libraries(hebi_gripper_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

install(
  TARGETS 
    hebi_turret_node 
    hebi_arm_node
    hebi_gripper_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_stick_bot_arm.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
