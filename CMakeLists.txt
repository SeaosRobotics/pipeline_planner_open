cmake_minimum_required(VERSION 2.8.3)
project(pipeline_planner)

#set(USE_GPU TRUE)
set(USE_GPU FALSE)
add_definitions(-DMODE_CPU=1)
add_definitions(-DMODE_GPU=2)
if(USE_GPU)
 add_definitions(-DMODE=2)
 find_package(CUDA REQUIRED)
 set(CUDA_NVCC_FLAGS "-arch=compute_30" CACHE STRING "nvcc flags" FORCE)
 set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE) 
 set(LIB_TYPE STATIC)
else()
 add_definitions(-DMODE=1)
endif()

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  std_msgs
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  move_base_msgs
  roscpp
  tf
  dynamic_reconfigure
  message_generation
)

add_message_files(
 DIRECTORY msg
 FILES
 RobotPosition.msg
 SegmentInfo.msg)

add_service_files(
 FILES
 GetReadStatus.srv
 GetRobotStatus.srv
 GetCheckpoints.srv
 GetNumofCheckpoints.srv
 InquireSegments.srv
 ReceiveCheckpoints.srv
 SetARadius.srv
 SetARightshift.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/PipelinePlanner.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pipeline_planner
  CATKIN_DEPENDS costmap_2d std_msgs geometry_msgs nav_core nav_msgs navfn move_base_msgs roscpp tf dynamic_reconfigure message_runtime
  #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(USE_GPU)

#cuda_add_library(ppplannerCUDALib ${LIB_TYPE} src/ppplanner_cudalib.cu)
 cuda_add_library(${PROJECT_NAME}
   src/ppplanner_core.cpp
   src/ppplanner_cudalib.cu
 )
#add_library(${PROJECT_NAME}
#  src/ppplanner_core.cpp
#)
 add_dependencies(${PROJECT_NAME}
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
#ppplannerCUDALib
 )
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
#  ppplannerCUDALib
 )
 add_executable(${PROJECT_NAME}_node src/ppplanner_node.cpp)
 add_dependencies(${PROJECT_NAME}_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS}
#ppplannerCUDALib
 )
 target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
#  ppplannerCUDALib
 )

else()

 add_library(${PROJECT_NAME}
   src/ppplanner_core.cpp
 )
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )
 add_executable(${PROJECT_NAME}_node src/ppplanner_node.cpp)
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

endif()

