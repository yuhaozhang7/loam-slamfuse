cmake_minimum_required(VERSION 2.8)
project(oh_my_loam)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(SLAMBENCH_INCLUDE_DIRS "/home/yuhao/slambench/framework/shared/include")

find_package(Ceres REQUIRED)
find_package(PCL QUIET)
find_package(g3log REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${G3LOG_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${SLAMBENCH_INCLUDE_DIRS}
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(common)
add_subdirectory(oh_my_loam)
add_subdirectory(examples)

set(appname loam)

add_library(${appname} SHARED
common/color/color.h
common/config/yaml_config.h
common/geometry/pose3d.h
common/geometry/trajectory.h
common/log/log.h 
common/macro/macros.h 
common/math/fitting.h 
common/math/math_utils.h 
common/pcl/pcl_types.h 
common/pcl/pcl_utils.h 
common/registerer/registerer.h 
common/time/timer.h 
common/visualizer/lidar_visualizer_utils.h 
common/visualizer/lidar_visualizer.h
common/common.h 
oh_my_loam/base/types.h 
oh_my_loam/base/feature.h 
oh_my_loam/base/utils.h 
oh_my_loam/extractor/extractor.h 
oh_my_loam/extractor/extractor_VLP.h 
oh_my_loam/extractor/extractor_VLP16.h
oh_my_loam/mapper/map.h 
oh_my_loam/mapper/mapper.h 
oh_my_loam/odometer/odometer.h 
oh_my_loam/solver/cost_function.h 
oh_my_loam/solver/solver.h 
oh_my_loam/visualizer/extractor_visualizer.h 
oh_my_loam/visualizer/mapper_visualizer.h 
oh_my_loam/visualizer/odometer_visualizer.h 
oh_my_loam/visualizer/ohmyloam_visualizer.h
oh_my_loam/oh_my_loam.h
common/config/yaml_config.cc
common/log/log.cc 
common/math/math_utils.cc 
common/time/timer.cc 
common/visualizer/lidar_visualizer_utils.cc 
oh_my_loam/base/utils.cc 
oh_my_loam/extractor/extractor.cc 
oh_my_loam/extractor/extractor_VLP.cc 
oh_my_loam/extractor/extractor_VLP16.cc 
oh_my_loam/mapper/map.cc 
oh_my_loam/mapper/mapper.cc 
oh_my_loam/odometer/odometer.cc 
oh_my_loam/solver/solver.cc 
oh_my_loam/visualizer/extractor_visualizer.cc 
oh_my_loam/visualizer/mapper_visualizer.cc 
oh_my_loam/visualizer/odometer_visualizer.cc 
oh_my_loam/visualizer/ohmyloam_visualizer.cc
oh_my_loam/oh_my_loam.cc)


##
add_executable(test test.cpp)

target_link_libraries(test 
  ${PCL_LIBRARIES}
  ${G3LOG_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${appname}
  ${CERES_LIBRARIES})
##

add_library(${appname}-original SHARED loam_library.cpp)

target_link_libraries(${appname}-original 
  ${PCL_LIBRARIES}
  ${G3LOG_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${appname}
  ${CERES_LIBRARIES})
        
SET(SLAMBENCH_C_WRAPPER	-Wl,--whole-archive -L/home/yuhao/slambench/build/lib slambench-c-wrapper	-Wl,--no-whole-archive)
SET(NULL_FILE_PATH	/home/yuhao/slambench/build/null.cpp)

function(generate_slam_library library_name )

    add_library(${library_name}-library SHARED ${NULL_FILE_PATH})
    target_link_libraries(${library_name}-library PRIVATE ${SLAMBENCH_C_WRAPPER} ${library_name})
    SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES LINK_FLAGS "${ARGN}")
    # SET_TARGET_PROPERTIES(${library_name}-library PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
    install(TARGETS ${library_name}-library  DESTINATION  lib/)

endfunction()

generate_slam_library(${appname}-original "")
