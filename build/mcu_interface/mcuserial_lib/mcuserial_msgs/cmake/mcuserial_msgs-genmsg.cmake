# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mcuserial_msgs: 2 messages, 1 services")

set(MSG_I_FLAGS "-Imcuserial_msgs:/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mcuserial_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg" NAME_WE)
add_custom_target(_mcuserial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mcuserial_msgs" "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg" ""
)

get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv" NAME_WE)
add_custom_target(_mcuserial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mcuserial_msgs" "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv" ""
)

get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg" NAME_WE)
add_custom_target(_mcuserial_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mcuserial_msgs" "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mcuserial_msgs
)
_generate_msg_cpp(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mcuserial_msgs
)

### Generating Services
_generate_srv_cpp(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mcuserial_msgs
)

### Generating Module File
_generate_module_cpp(mcuserial_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mcuserial_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mcuserial_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mcuserial_msgs_generate_messages mcuserial_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_cpp _mcuserial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_cpp _mcuserial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_cpp _mcuserial_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mcuserial_msgs_gencpp)
add_dependencies(mcuserial_msgs_gencpp mcuserial_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mcuserial_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mcuserial_msgs
)
_generate_msg_eus(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mcuserial_msgs
)

### Generating Services
_generate_srv_eus(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mcuserial_msgs
)

### Generating Module File
_generate_module_eus(mcuserial_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mcuserial_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mcuserial_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mcuserial_msgs_generate_messages mcuserial_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_eus _mcuserial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_eus _mcuserial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_eus _mcuserial_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mcuserial_msgs_geneus)
add_dependencies(mcuserial_msgs_geneus mcuserial_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mcuserial_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mcuserial_msgs
)
_generate_msg_lisp(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mcuserial_msgs
)

### Generating Services
_generate_srv_lisp(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mcuserial_msgs
)

### Generating Module File
_generate_module_lisp(mcuserial_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mcuserial_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mcuserial_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mcuserial_msgs_generate_messages mcuserial_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_lisp _mcuserial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_lisp _mcuserial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_lisp _mcuserial_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mcuserial_msgs_genlisp)
add_dependencies(mcuserial_msgs_genlisp mcuserial_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mcuserial_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mcuserial_msgs
)
_generate_msg_nodejs(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mcuserial_msgs
)

### Generating Services
_generate_srv_nodejs(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mcuserial_msgs
)

### Generating Module File
_generate_module_nodejs(mcuserial_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mcuserial_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mcuserial_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mcuserial_msgs_generate_messages mcuserial_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_nodejs _mcuserial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_nodejs _mcuserial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_nodejs _mcuserial_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mcuserial_msgs_gennodejs)
add_dependencies(mcuserial_msgs_gennodejs mcuserial_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mcuserial_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mcuserial_msgs
)
_generate_msg_py(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mcuserial_msgs
)

### Generating Services
_generate_srv_py(mcuserial_msgs
  "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mcuserial_msgs
)

### Generating Module File
_generate_module_py(mcuserial_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mcuserial_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mcuserial_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mcuserial_msgs_generate_messages mcuserial_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/TopicInfo.msg" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_py _mcuserial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/srv/RequestParam.srv" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_py _mcuserial_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rokot1/design4_workspace/src/mcu_interface/mcuserial_lib/mcuserial_msgs/msg/Log.msg" NAME_WE)
add_dependencies(mcuserial_msgs_generate_messages_py _mcuserial_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mcuserial_msgs_genpy)
add_dependencies(mcuserial_msgs_genpy mcuserial_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mcuserial_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mcuserial_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mcuserial_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mcuserial_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mcuserial_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mcuserial_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mcuserial_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mcuserial_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mcuserial_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mcuserial_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mcuserial_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mcuserial_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
