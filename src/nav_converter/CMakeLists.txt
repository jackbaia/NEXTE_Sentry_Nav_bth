cmake_minimum_required(VERSION 3.0.2)
project(nav_converter)

find_package(Eigen3 REQUIRED)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PositionCommand.msg
  MavlinkPositionCommand.msg
)

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
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    tf 
    message_runtime
  DEPENDS
    message_generation
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(cmd_vel_to_position src/cmd_vel_to_position.cpp)
add_executable(path_following_controller src/path_following_controller.cpp)
add_executable(advanced_path_controller src/advanced_path_controller.cpp)
add_executable(tunnel_path_controller src/tunnel_path_controller.cpp)
add_executable(mavlink_position_controller src/mavlink_position_controller.cpp)
add_executable(coordinate_transformer src/coordinate_transformer.cpp)

## Add cmake target dependencies of the executable
add_dependencies(cmd_vel_to_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(path_following_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(advanced_path_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(tunnel_path_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(mavlink_position_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(coordinate_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cmd_vel_to_position
  ${catkin_LIBRARIES}
)
target_link_libraries(path_following_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(advanced_path_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(tunnel_path_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(mavlink_position_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(coordinate_transformer
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS cmd_vel_to_position path_following_controller advanced_path_controller tunnel_path_controller mavlink_position_controller coordinate_transformer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/nav_converter.launch
  launch/path_following_controller.launch
  launch/tunnel_path_controller.launch
  launch/mavlink_position_controller.launch
  param/converter_params.yaml
  param/tunnel_controller_params.yaml
  param/mavlink_controller_params.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark Python scripts for installation
install(PROGRAMS
  scripts/test_converter.py
  scripts/test_tunnel_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nav_converter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test) 