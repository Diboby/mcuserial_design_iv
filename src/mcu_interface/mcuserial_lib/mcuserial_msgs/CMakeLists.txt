cmake_minimum_required(VERSION 3.7.2)
project(mcuserial_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(FILES
# Il faut rajouter les messages crees ici
  Log.msg
  TopicInfo.msg
)

add_service_files(FILES
  alim_serial_com_srv.srv
)

generate_messages()

catkin_package(CATKIN_DEPENDS
  message_runtime
)
