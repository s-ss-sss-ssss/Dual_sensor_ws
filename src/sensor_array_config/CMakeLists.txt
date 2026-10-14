cmake_minimum_required(VERSION 3.0.2)
project(sensor_array_config)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

catkin_python_setup()

catkin_package()

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
