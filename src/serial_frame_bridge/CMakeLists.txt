cmake_minimum_required(VERSION 3.0.2)
project(serial_frame_bridge)

add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs diagnostic_msgs message_generation)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(FILES RawFrame.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(INCLUDE_DIRS include LIBRARIES serial_frame_bridge_core
               CATKIN_DEPENDS roscpp std_msgs diagnostic_msgs message_runtime
               DEPENDS Boost)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(serial_frame_bridge_core src/frame_parser.cpp src/serial_bridge.cpp)
target_link_libraries(serial_frame_bridge_core ${Boost_LIBRARIES})
add_dependencies(serial_frame_bridge_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(serial_frame_bridge_node src/bridge_node.cpp)
add_dependencies(serial_frame_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_frame_bridge_node serial_frame_bridge_core ${catkin_LIBRARIES})

install(TARGETS serial_frame_bridge_core serial_frame_bridge_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_frame_parser test/test_frame_parser.cpp)
  if(TARGET test_frame_parser)
    target_link_libraries(test_frame_parser serial_frame_bridge_core ${catkin_LIBRARIES} gtest_main)
  endif()
  if(UNIX AND NOT APPLE)
    catkin_add_gtest(test_serial_pty test/test_serial_pty.cpp)
    if(TARGET test_serial_pty)
      target_link_libraries(test_serial_pty serial_frame_bridge_core ${catkin_LIBRARIES} gtest_main util)
    endif()
  endif()
endif()
