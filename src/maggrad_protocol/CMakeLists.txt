cmake_minimum_required(VERSION 3.0.2)
project(maggrad_protocol)

add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs diagnostic_msgs serial_frame_bridge message_generation)

add_message_files(FILES AkSample.msg TmagSample.msg IcmSample.msg ErrorRecord.msg StatsRecord.msg RoundSample.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(INCLUDE_DIRS include LIBRARIES maggrad_decoder
  CATKIN_DEPENDS roscpp std_msgs diagnostic_msgs serial_frame_bridge message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(maggrad_decoder src/decoder.cpp)
add_dependencies(maggrad_decoder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(protocol_node src/protocol_node.cpp)
add_dependencies(protocol_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(protocol_node maggrad_decoder ${catkin_LIBRARIES})

install(TARGETS maggrad_decoder protocol_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_executable(test_maggrad_decoder test/test_decoder.cpp)
  target_compile_options(test_maggrad_decoder PRIVATE -UNDEBUG)
  target_link_libraries(test_maggrad_decoder maggrad_decoder)
  add_test(NAME test_maggrad_decoder COMMAND test_maggrad_decoder)
endif()
