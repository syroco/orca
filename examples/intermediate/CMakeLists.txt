add_executable(01-using_callbacks 01-using_callbacks.cc)
target_link_libraries(01-using_callbacks orca)
install(TARGETS 01-using_callbacks DESTINATION bin/orca/examples)
