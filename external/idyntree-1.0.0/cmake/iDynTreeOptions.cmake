#########################################################################
# Control whether libraries are shared or static.
if( MSVC )
option(IDYNTREE_SHARED_LIBRARY "Compile iDynTree as a shared library" FALSE)
else()
option(IDYNTREE_SHARED_LIBRARY "Compile iDynTree as a shared library" TRUE)
endif()
set(BUILD_SHARED_LIBS ${IDYNTREE_SHARED_LIBRARY})

#########################################################################
option(IDYNTREE_ONLY_DOCS "Only produce iDynTree documentation, without compiling" FALSE)
mark_as_advanced(IDYNTREE_ONLY_DOCS)

#########################################################################
# Use position indipendent code
set (CMAKE_POSITION_INDEPENDENT_CODE TRUE)

#########################################################################
# Turn on testing.
option(IDYNTREE_COMPILE_TESTS "Compile iDynTree tests" FALSE)
option(IDYNTREE_RUN_VALGRIND_TESTS "Run iDynTree tests with Valgrind" FALSE)
mark_as_advanced(IDYNTREE_RUN_VALGRIND_TESTS)
if(IDYNTREE_COMPILE_TESTS)
   include( CTest )
   enable_testing()

   # adding support for checking the tests with valgrind
   if(IDYNTREE_RUN_VALGRIND_TESTS)
        find_package(Valgrind REQUIRED)
        if(VALGRIND_FOUND)
            set(CTEST_MEMORYCHECK_COMMAND ${VALGRIND_PROGRAM})
            set(MEMORYCHECK_COMMAND ${VALGRIND_PROGRAM})
            if (APPLE)
                set(MEMORYCHECK_SUPPRESSIONS "--suppressions=${PROJECT_SOURCE_DIR}/cmake/valgrind-macos.supp")
            else ()
                set(MEMORYCHECK_SUPPRESSIONS "")
            endif ()
            set(MEMORYCHECK_COMMAND_OPTIONS "--leak-check=full --error-exitcode=1 ${MEMORYCHECK_SUPPRESSIONS}"  CACHE STRING "Options to pass to the memory checker")
            mark_as_advanced(MEMORYCHECK_COMMAND_OPTIONS)
            set(MEMCHECK_COMMAND_COMPLETE "${MEMORYCHECK_COMMAND} ${MEMORYCHECK_COMMAND_OPTIONS}")
            separate_arguments(MEMCHECK_COMMAND_COMPLETE)
        endif()
   endif()
endif()

#########################################################################
# Turn on compilation of geometrical relations semantics check.
option(IDYNTREE_USES_SEMANTICS "Compile iDynTree semantics check" FALSE)


#########################################################################
# Turn off compilation of optimal control part.
option(IDYNTREE_COMPILES_OPTIMALCONTROL "Compile iDynTree optimal control part." TRUE)

#########################################################################
# Deal with RPATH
option(IDYNTREE_ENABLE_RPATH "Enable RPATH for the library" TRUE)
mark_as_advanced(IDYNTREE_ENABLE_RPATH)

if( MSVC )
    option(IDYNTREE_USES_INTERNAL_URDFDOM "Compile iDynTree with an internal copy of urdfdom patched to avoid Boost dependencies" TRUE)
else()
    option(IDYNTREE_USES_INTERNAL_URDFDOM "Compile iDynTree with an internal copy of urdfdom patched to avoid Boost dependencies" FALSE)
endif()

if(IDYNTREE_USES_INTERNAL_URDFDOM)
    add_definitions(-DURDF_USE_PLAIN_POINTERS)
    add_definitions(-DIDYNTREE_USE_INTERNAL_URDFDOM)
endif()

##########################################################################
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

# Compile flags definitions for Semantic checks
if(IDYNTREE_COMPILE_BINDINGS)
    add_definitions(-DIDYNTREE_COMPILE_BINDINGS)
endif(IDYNTREE_COMPILE_BINDINGS)

if(IDYNTREE_USES_SEMANTICS)
    add_definitions(-DIDYNTREE_USES_SEMANTICS)
endif(IDYNTREE_USES_SEMANTICS)

#set default build type to "Release" in single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, recommanded options are: Debug or Release" FORCE)
    endif()
    set(IDYNTREE_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${IDYNTREE_BUILD_TYPES})
endif()

# Include ECM Sanitizers, for enable compilation support of GCC and Clang
# sanitizers. Imported from http://api.kde.org/ecm/module/ECMEnableSanitizers.html
include(ECMEnableSanitizers)

# Enable warnings if requested

# Save compiler specific warnings flags
if((${CMAKE_CXX_COMPILER_ID} MATCHES "GNU") OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
    set(IDYNTREE_WARNING_FLAGS "")
    list(APPEND IDYNTREE_WARNING_FLAGS -Wall)
    list(APPEND IDYNTREE_WARNING_FLAGS -Wextra)
    list(APPEND IDYNTREE_WARNING_FLAGS -Woverloaded-virtual)
    list(APPEND IDYNTREE_WARNING_FLAGS -pedantic)
endif()

# Create build artifacts in ${build}/bin for binaries and ${build}/lib for libraries
# This simplifies the use of isaac software directly from the build directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

include(CheckCXXCompilerFlag)

# Macro to remove deprecation warnings for a specific part of iDynTree
macro(idyntree_disable_deprecation_warnings)
  if(MSVC)
    add_definitions(/wd4996)
  else()
    CHECK_CXX_COMPILER_FLAG("-Wno-deprecated-declarations" CXX_HAS_WNO_DEPRECATED_DECLARATIONS)
    if(CXX_HAS_WNO_DEPRECATED_DECLARATIONS)
      add_definitions(-Wno-deprecated-declarations)
    endif()
    CHECK_CXX_COMPILER_FLAG("-Wno-deprecated" CXX_HAS_WNO_DEPRECATED)
    if(CXX_HAS_WNO_DEPRECATED)
      add_definitions(-Wno-deprecated)
    endif()
  endif()
endmacro()
