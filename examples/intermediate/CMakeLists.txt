add_executable(01-using_callbacks 01-using_callbacks.cc)
target_link_libraries(01-using_callbacks orca Eigen3::Eigen)
install(TARGETS 01-using_callbacks DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)

add_executable(02-using_lambda_callbacks 02-using_lambda_callbacks.cc)
target_link_libraries(02-using_lambda_callbacks orca Eigen3::Eigen)
install(TARGETS 02-using_lambda_callbacks DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/examples)
