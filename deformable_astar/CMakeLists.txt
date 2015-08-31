cmake_minimum_required(VERSION 2.8.3)
project(deformable_astar)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS arc_utilities roscpp rospy roslib geometry_msgs message_generation std_msgs tf xtf trajectory_msgs arc_utilities sdf_tools)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
  FILES
  DeformationState.msg
  PathState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY
  srv
  FILES
  CameraTrigger.srv
  DeformPlanner.srv
  GRCQuery.srv
  SDFQuery.srv
  PerObjectCostQuery.srv
  CostQuery.srv
  CalibrateCost.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  trajectory_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES deformable_astar
  CATKIN_DEPENDS arc_utilities rospy roscpp geometry_msgs message_runtime std_msgs roslib tf xtf trajectory_msgs arc_utilities sdf_tools
  DEPENDS system_lib Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} SYSTEM ${Eigen_INCLUDE_DIRS})

## Set build flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -O3 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3 -g -Wall -Wextra")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")

## Declare cpp libraries
add_library(${PROJECT_NAME} src/dVoxelLib.cpp src/SearchTools.cpp src/SearchFramework.cpp src/Astar.cpp src/IrosPlanner.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} rt)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

## Declare cpp executables
add_executable(planner3d src/main.cpp)
target_link_libraries(planner3d ${PROJECT_NAME} ${catkin_LIBRARIES} rt)
add_dependencies(planner3d ${PROJECT_NAME}_gencpp)

add_executable(planner2d src/planner2d.cpp)
target_link_libraries(planner2d ${PROJECT_NAME} ${catkin_LIBRARIES} rt)
add_dependencies(planner2d ${PROJECT_NAME}_gencpp)

add_executable(statechecker src/standaloneEvaluator.cpp)
target_link_libraries(statechecker ${PROJECT_NAME} ${catkin_LIBRARIES} rt)
add_dependencies(statechecker ${PROJECT_NAME}_gencpp)

add_executable(plannernode src/PlannerNode.cpp)
target_link_libraries(plannernode ${PROJECT_NAME} ${catkin_LIBRARIES} rt)
add_dependencies(plannernode ${PROJECT_NAME}_gencpp)

add_executable(visualizernode src/VisualizerNode.cpp)
target_link_libraries(visualizernode ${PROJECT_NAME} ${catkin_LIBRARIES} rt)
add_dependencies(visualizernode ${PROJECT_NAME}_gencpp)

add_executable(xtf_sampler src/xtf_sampler_node.cpp)
target_link_libraries(xtf_sampler ${PROJECT_NAME} ${catkin_LIBRARIES} rt)
add_dependencies(xtf_sampler ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

add_executable(voxel_openmp_test src/voxel_openmp_test_node.cpp)
target_link_libraries(voxel_openmp_test ${PROJECT_NAME} ${catkin_LIBRARIES} rt)
add_dependencies(voxel_openmp_test ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(deformable_astar_node deformable_astar_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(deformable_astar_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS deformable_astar deformable_astar_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_deformable_astar.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
