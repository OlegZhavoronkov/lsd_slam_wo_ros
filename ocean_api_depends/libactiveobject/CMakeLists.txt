#if( NOT FIPS_IMPORT )
  cmake_minimum_required(VERSION 3.5)

#  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
#  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")
project(libactiveobject LANGUAGES C CXX)
#  fips_setup(PROJECT libactiveobject)
#endif()

# 	# A few things which are otherwise handled by Conan
#  if( APPLE )
#     SET(CMAKE_INSTALL_RPATH "@executable_path/lib")
#     set (MACOSX_RPATH ON)
#   endif()
#
# 	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
# 	set(RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
# endif()
#
#
# find_package(Threads)

find_package(Boost COMPONENTS thread filesystem system)

## C++ flags applied to all builds
#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Set global include paths
set(LIBACTIVE_OBJECT_INCLUDE_DIRS_LOCAL ${CMAKE_CURRENT_SOURCE_DIR}/include )
set( LIBACTIVE_OBJECT_INCLUDE_DIRS ${LIBACTIVE_OBJECT_INCLUDE_DIRS_LOCAL} PARENT_SCOPE)
include_directories( ${LIBACTIVE_OBJECT_INCLUDE_DIRS_LOCAL} )

add_subdirectory("lib/")

#if( NOT FIPS_IMPORT )
#
#  if( FIPS_UNITTESTS )
#    message("** Will run unit tests")
#    fips_add_subdirectory( test/unit/ )
#  endif()
#
#  fips_finish()
#endif()
