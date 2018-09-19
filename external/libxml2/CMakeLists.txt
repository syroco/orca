cmake_minimum_required(VERSION 3.1)
project(xml2 C)

set(xml2_dir ${PROJECT_SOURCE_DIR}/libxml2-2.9.8)

file(GLOB srcs ${xml2_dir}/*.c)
list(FILTER srcs EXCLUDE REGEX "/(run|test|trio)[^/]*$")
list(FILTER srcs EXCLUDE REGEX "xml(lint|catalog).c$")

# Generate xmlexports with fixed definition of LIBXML_STATIC
file(READ ${xml2_dir}/include/libxml/xmlexports.h XMLEXPORTS_H)
string(REPLACE "!defined(LIBXML_STATIC)" "0" XMLEXPORTS_H "${XMLEXPORTS_H}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/xmlexports.h "${XMLEXPORTS_H}")

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    configure_file(${xml2_dir}/include/win32config.h ${CMAKE_CURRENT_BINARY_DIR}/config.h COPYONLY)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    configure_file(${PROJECT_SOURCE_DIR}/config-macos.h ${CMAKE_CURRENT_BINARY_DIR}/config.h COPYONLY)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    configure_file(${PROJECT_SOURCE_DIR}/config-linux.h ${CMAKE_CURRENT_BINARY_DIR}/config.h COPYONLY)
endif()

add_library(xml2 STATIC ${srcs})
target_include_directories(xml2 PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${xml2_dir}/include)

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_include_directories(xml2 PRIVATE ${xml2_dir}/win32/vc10)
    target_link_libraries(xml2 PRIVATE wsock32.lib ws2_32.lib)
    target_compile_definitions(xml2 PRIVATE -DHAVE_WIN32_THREADS)
endif()

target_compile_definitions(xml2 PRIVATE -D_REENTRANT)
