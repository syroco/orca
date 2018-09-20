cmake_minimum_required(VERSION 3.1)
project(libxml2)

set(xml2_top_dir ${PROJECT_SOURCE_DIR})
set(xml2_dir ${xml2_top_dir}/libxml2-2.9.8)

file(GLOB srcs ${xml2_dir}/*.c)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    configure_file(${xml2_top_dir}/include/win32config.h ${CMAKE_CURRENT_BINARY_DIR}/config.h COPYONLY)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    configure_file(${xml2_top_dir}/config-macos.h ${CMAKE_CURRENT_BINARY_DIR}/config.h COPYONLY)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    configure_file(${xml2_top_dir}/config-linux.h ${CMAKE_CURRENT_BINARY_DIR}/config.h COPYONLY)
endif()

add_library(xml2 STATIC ${srcs})
target_include_directories(xml2 PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${xml2_dir} ${xml2_dir}/include)
set(LIB ${xml2_dir}/include)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_include_directories(xml2 PRIVATE ${xml2_dir}/win32/vc10)
    target_link_libraries(xml2 PRIVATE wsock32.lib ws2_32.lib)
    target_compile_definitions(xml2 PRIVATE -DHAVE_WIN32_THREADS)
endif()

target_compile_definitions(xml2 PRIVATE -D_REENTRANT)
# Hack to fix iDynTree export set
install(TARGETS xml2 EXPORT iDynTree DESTINATION lib)
set_property(GLOBAL APPEND PROPERTY iDynTree_TARGETS xml2)