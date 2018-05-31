add_executable(01-using_callbacks 01-using_callbacks.cc)
target_link_libraries(01-using_callbacks orca)
install(TARGETS 01-using_callbacks DESTINATION bin/orca/examples)

add_executable(02-using_lambda_callbacks 02-using_lambda_callbacks.cc)
target_link_libraries(02-using_lambda_callbacks orca)
install(TARGETS 02-using_lambda_callbacks DESTINATION bin/orca/examples)
