cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)

find_package(  catkin   REQUIRED 
  roscpp 
  rospy 
  turtlesim 
  std_msgs 
  message_generation
  tf
  costmap_2d
  global_planner
  base_local_planner
  rotate_recovery
  dwa_local_planner
)

add_service_files(
  FILES
  STPT.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  turtlesim
)
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS message_runtime
   DEPENDS elektron 
)

include_directories(include
	${catkin_INCLUDE_DIRS})

add_executable(maldito_planificador_local src/maldito_planificador_local.cpp)
target_link_libraries(maldito_planificador_local ${catkin_LIBRARIES})

install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
