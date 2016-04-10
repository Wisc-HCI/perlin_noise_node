cmake_minimum_required(VERSION 2.8.3)
project(perlin_noise_node)

##############################################################################
# Find dependencies
##############################################################################

## Declare main C++ dependencies
#set(ROS_CXX_DEPENDENCIES
#  naoqi_bridge_msgs
#  roscpp
#  std_msgs
#  std_srvs
#  geometry_msgs
#  angles
#)

set(THIS_PACKAGE_ROS_DEPS
 dynamic_reconfigure
 message_generation
 roscpp
 std_msgs
 filters
 perlin_noise_filter
)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})

add_message_files(
  FILES
  NodeExampleData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/nodeExample.cfg
)

##############################################################################
# Define package
##############################################################################

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp std_msgs filters perlin_noise_filter
)

##############################################################################
# Build
##############################################################################

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp src/nodes/talker_node.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker node_example_gencfg node_example_generate_messages_cpp)
add_executable(listener src/listener.cpp src/nodes/listener_node.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener node_example_gencfg node_example_generate_messages_cpp)

##############################################################################
# Install
##############################################################################

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS talker listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
