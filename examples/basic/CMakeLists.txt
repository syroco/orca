add_executable(01-simple_controller 01-simple_controller.cc)
target_link_libraries(01-simple_controller orca)
install(TARGETS 01-simple_controller DESTINATION bin/orca/examples)
