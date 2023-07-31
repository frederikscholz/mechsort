# cmake_minimum_required(VERSION 2.8.3)
# project(mechsort)

# ## Compile as C++11, supported in ROS Kinetic and newer
# #add_compile_options(-std=c++11)

# ## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   ##sensor_msgs
#   pcl_conversions
#   pcl_ros
#   geometry_msgs
#   moveit_core
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   message_generation 
# )

# ## System dependencies are found with CMake's conventions
# add_message_files(
# #   # DIRECTORY
# #   # pose_msg/src/msg
#    FILES
# #   # list all the .msg files in the pose_msg package
#    pose.msg
# )
# # Add service files
# add_service_files(
#   FILES
#   pose.srv
# )
# ##find_package(PCL 1.7 REQUIRED)
# generate_messages(
#   DEPENDENCIES
#   # list any other message or service dependencies
#   std_msgs
# )


# ## Declare catkin dependencies
# catkin_package(
#   CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros geometry_msgs message_runtime 
# )
# ##pose_msg
# ## Specify additional locations of header files
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ##${PCL_INCLUDE_DIRS}
# )

# ## Declare a C++ executable
# #add_executable(highest_point_node src/highest_point_node.cpp)
# add_executable(pc_receiver_node src/pc_receiver.cpp)
# add_executable(sia10f_move_node src/sia10f_move_node.cpp)
# add_executable(pose_server_node src/pose_server.cpp)



# add_dependencies(pose_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# ## Specify libraries to link a library or executable target against
# target_link_libraries(pc_receiver_node
#   ${catkin_LIBRARIES}
#   ${MOVEIT_LIBRARIES}
#   ##${PCL_LIBRARIES}
# )
# target_link_libraries(sia10f_move_node
#   ${catkin_LIBRARIES}
#   ${MOVEIT_LIBRARIES}
#   ##${PCL_LIBRARIES}
# )
# target_link_libraries(pose_server_node
#   ${catkin_LIBRARIES}
#   ${MOVEIT_LIBRARIES}
#   ##${PCL_LIBRARIES}
# )
cmake_minimum_required(VERSION 2.8.3)
project(mechsort)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ##sensor_msgs
  pcl_conversions
  pcl_ros
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  message_generation 
  pose_msg
)

## System dependencies are found with CMake's conventions
add_message_files(
  FILES
  #pose_msg/pose.msg
)

# Add service files
add_service_files(
  FILES
  pose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  # list any other message or service dependencies
  ## Add other message dependencies if needed
)

## Declare catkin dependencies
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros geometry_msgs message_runtime 
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
#add_executable(highest_point_node src/highest_point_node.cpp)
add_executable(pc_receiver_node src/pc_receiver.cpp)
add_executable(sia10f_move_node src/sia10f_move_node.cpp)
# add_executable(pose_server_node src/pose_server.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pc_receiver_node
  ${catkin_LIBRARIES}
  ${MOVEIT_LIBRARIES}
)

target_link_libraries(sia10f_move_node
  ${catkin_LIBRARIES}
  ${MOVEIT_LIBRARIES}
)

# target_link_libraries(pose_server_node pose_msg
#   ${catkin_LIBRARIES}
#   ${MOVEIT_LIBRARIES}
# )

# Add dependencies to the generate_messages target
add_dependencies(pc_receiver_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(sia10f_move_node ${PROJECT_NAME}_generate_messages_cpp)
#add_dependencies(pose_server_node ${PROJECT_NAME}_generate_messages_cpp)
# add_dependencies(pose_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) # Add this line for message generation
