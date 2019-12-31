cmake_minimum_required(VERSION 2.8.3)
project(mk_odometry)
find_package(catkin REQUIRED
            rospy
            std_msgs
            message_generation)

add_message_files(
  FILES
  Encoder.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
