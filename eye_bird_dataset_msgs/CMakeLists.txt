cmake_minimum_required(VERSION 3.0.2)
project(eye_bird_dataset_msgs)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

# search for all msg files
FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

# notify catkin to look at the previously found msg files
add_message_files(
  FILES
  ${messages_to_build}
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
