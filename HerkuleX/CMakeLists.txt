cmake_minimum_required(VERSION 2.8.3)
project(HerkuleX)
add_definitions(-std=c++11) # Use C++11
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  sensor_msgs
  message_generation
  message_runtime
  tf
  nav_msgs
  geometry_msgs
)
find_package(Eigen3 REQUIRED)
add_message_files(
	FILES 
	MsgHerkuleX_RAM.msg
	MsgHerkuleX_EEP.msg
	HerkuleX_Info_RAM.msg
	HerkuleX_Info_EEP.msg
)
add_service_files(
 	FILES
	HerkuleX_RegisterCommand.srv
	HerkuleX_PositionMove.srv
	HerkuleX_VelocityMove.srv
	HerkuleX_SJOG_Move.srv
	HerkuleX_IJOG_Move.srv	
)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    DEPENDS EIGEN3
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
#HerkuleX
add_executable(HerkuleX_node src/HerkuleX_node.cpp)
add_dependencies(HerkuleX_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HerkuleX_node ${catkin_LIBRARIES})
#HerkuleX_feedback_Test_node
add_executable(HerkuleX_feedback_Test_node src/HerkuleX_feedback_Test_node.cpp)
add_dependencies(HerkuleX_feedback_Test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HerkuleX_feedback_Test_node ${catkin_LIBRARIES})
#HerkuleX_PanTilt_node
add_executable(HerkuleX_PanTilt_node src/HerkuleX_PanTilt_node.cpp)
add_dependencies(HerkuleX_PanTilt_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HerkuleX_PanTilt_node ${catkin_LIBRARIES})
#HerkuleX_12DOF_Robot_node
add_executable(HerkuleX_12DOF_Robot_node src/HerkuleX_12DOF_Robot_node.cpp)
add_dependencies(HerkuleX_12DOF_Robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HerkuleX_12DOF_Robot_node ${catkin_LIBRARIES})
#HerkuleX_6DOF_Arm_node
add_executable(HerkuleX_6DOF_Arm_node src/HerkuleX_6DOF_Arm_node.cpp)
add_dependencies(HerkuleX_6DOF_Arm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HerkuleX_6DOF_Arm_node ${catkin_LIBRARIES})





