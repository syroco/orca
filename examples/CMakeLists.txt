add_executable(orca-demo0 orca-demo0.cc)
target_link_libraries(orca-demo0 orca)

# add_executable(orca-demo1 orca-demo1.cc)
# target_link_libraries(orca-demo1 orca)

# find_package(gazebo QUIET)
# 
# if(gazebo_FOUND)
#     if(${GAZEBO_VERSION} VERSION_GREATER "7.0")
#         message(STATUS "Gazebo ${GAZEBO_VERSION} found, building examples")
# 
#         include_directories(${GAZEBO_INCLUDE_DIRS})
#         link_directories(${GAZEBO_LIBRARY_DIRS})
#         list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -fext-numeric-literals")
# 
#         add_executable(orca-demo1-gazebo orca-demo1-gazebo.cc)
#         target_link_libraries(orca-demo1-gazebo ${GAZEBO_LIBRARIES} pthread orca tinyxml)
#         install(TARGETS orca-demo1-gazebo DESTINATION lib/orca)
#     else()
#         message(WARNING "Gazebo ${GAZEBO_VERSION} found, we need at least gazebo 7")
#     endif()
# else()
#     message(STATUS "Gazebo not found, not building gazebo examples")
# endif()

# install(TARGETS orca-demo1 DESTINATION lib/orca)
