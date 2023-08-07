add_executable(main_noros main_noros.cc)
target_link_libraries(main_noros
  ${PCL_LIBRARIES}
  ${G3LOG_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  common
  oh_my_loam
  extractor
  odometer
  mapper
  solver
  ${CERES_LIBRARIES}
  visualizer
  base
)