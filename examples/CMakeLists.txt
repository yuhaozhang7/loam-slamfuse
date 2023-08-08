add_executable(main_noros main_noros.cc)

add_library(common-lib SHARED
../common/color/color.h
../common/config/yaml_config.h
../common/geometry/pose3d.h
../common/geometry/trajectory.h
../common/log/log.h 
../common/macro/macros.h 
../common/math/fitting.h 
../common/math/math_utils.h 
../common/pcl/pcl_types.h 
../common/pcl/pcl_utils.h 
../common/registerer/registerer.h 
../common/time/timer.h 
../common/visualizer/lidar_visualizer_utils.h 
../common/visualizer/lidar_visualizer.h
../common/common.h 
../oh_my_loam/base/types.h 
../oh_my_loam/base/feature.h 
../oh_my_loam/base/utils.h 
../oh_my_loam/extractor/extractor.h 
../oh_my_loam/extractor/extractor_VLP.h 
../oh_my_loam/extractor/extractor_VLP16.h
../oh_my_loam/mapper/map.h 
../oh_my_loam/mapper/mapper.h 
../oh_my_loam/odometer/odometer.h 
../oh_my_loam/solver/cost_function.h 
../oh_my_loam/solver/solver.h 
../oh_my_loam/visualizer/extractor_visualizer.h 
../oh_my_loam/visualizer/mapper_visualizer.h 
../oh_my_loam/visualizer/odometer_visualizer.h 
../oh_my_loam/visualizer/ohmyloam_visualizer.h
../oh_my_loam/oh_my_loam.h
../common/config/yaml_config.cc
../common/log/log.cc 
../common/math/math_utils.cc 
../common/time/timer.cc 
../common/visualizer/lidar_visualizer_utils.cc 
../oh_my_loam/base/utils.cc 
../oh_my_loam/extractor/extractor.cc 
../oh_my_loam/extractor/extractor_VLP.cc 
../oh_my_loam/extractor/extractor_VLP16.cc 
../oh_my_loam/mapper/map.cc 
../oh_my_loam/mapper/mapper.cc 
../oh_my_loam/odometer/odometer.cc 
../oh_my_loam/solver/solver.cc 
../oh_my_loam/visualizer/extractor_visualizer.cc 
../oh_my_loam/visualizer/mapper_visualizer.cc 
../oh_my_loam/visualizer/odometer_visualizer.cc 
../oh_my_loam/visualizer/ohmyloam_visualizer.cc
../oh_my_loam/oh_my_loam.cc)

target_link_libraries(main_noros
  ${PCL_LIBRARIES}
  ${G3LOG_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  common-lib
  ${CERES_LIBRARIES}
)

# target_link_libraries(main_noros
#  ${PCL_LIBRARIES}
#  ${G3LOG_LIBRARIES}
#  ${YAML_CPP_LIBRARIES}
#  common
#  oh_my_loam
#  extractor
#  odometer
#  mapper
#  solver
#  ${CERES_LIBRARIES}
#  visualizer
#  base
#)
