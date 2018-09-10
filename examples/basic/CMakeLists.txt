add_executable(01-simple_controller 01-simple_controller.cc)
target_link_libraries(01-simple_controller orca)
install(TARGETS 01-simple_controller DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

add_executable(02-simulating_results 02-simulating_results.cc)
target_link_libraries(02-simulating_results orca)
install(TARGETS 02-simulating_results DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

add_executable(03-loading_from_file 03-loading_from_file.cc)
target_link_libraries(03-loading_from_file orca)
install(TARGETS 03-loading_from_file DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)
