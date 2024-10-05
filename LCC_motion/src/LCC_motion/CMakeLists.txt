cmake_minimum_required(VERSION 2.8.3)
project(lcc_motion)

SET(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++17 )
ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" ) 

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")
 
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
  cv_bridge
   
)
LIST( APPEND CMAKE_MODULE_PATH /home/wyw/ROS1_PROJECT/2023/camera_lidar_new/g2o/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o) 
set(G2O_ROOT /usr/local/include/g2o) 
find_package(G2O REQUIRED)
SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)
find_package(G2O REQUIRED)
 include_directories(${G2O_INCLUDE_DIRS})
  
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
message(Eigen: ${EIGEN3_INCLUDE_DIR})

 

include_directories(
     ../../devel/include
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
   ${PYTHON_INCLUDE_DIRS}
  include)
 include_directories(${CERES_INCLUDE_DIRS})
  include_directories(${YAML_INCLUDE_DIRS} ~/YOLOV7_ROS/devel/include/objdetection)
  include_directories(${YAML_INCLUDE_DIRS} ~/YOLOV7_ROS/devel/include/imu_motion_judge)
 
generate_messages(
 DEPENDENCIES
 
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
 
)

add_executable(topsic_correct
      
        src/topsic_correct.cpp
     
        )
target_link_libraries(topsic_correct  ${catkin_LIBRARIES} ${G2O_LIBS}  )


# link_directories(
#   include/g2o/lib # 相对路径，指定动态链接库的访问路径
#   ${catkin_LIB_DIRS}
# )
add_executable(LIcalib src/initial_ex_rotation.cpp)

target_link_libraries(LIcalib  ${catkin_LIBRARIES} ${G2O_LIBS}  
# g2o_core g2o_stuff 
)






