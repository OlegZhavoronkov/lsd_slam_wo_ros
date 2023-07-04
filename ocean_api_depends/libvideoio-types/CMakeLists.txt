cmake_minimum_required(VERSION 3.5)
project(libvideoio_types LANGUAGES C CXX)
#if( NOT FIPS_IMPORT )
#	get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
#	include("${FIPS_ROOT_DIR}/cmake/fips.cmake")
#
#	fips_setup(PROJECT libvideoio_types)
#endif()

find_package( Eigen3 REQUIRED )
#set(CMAKE_CXX_STANDARD 11)
#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

## Set global include paths

include_directories( include
					${EIGEN3_INCLUDE_DIR}
  				${OpenCV_INCLUDE_DIRS} )
if(LIBVIDEOIO-TYPES_INCLUDE STREQUAL ${INITIAL_LIBVIDEOIO-TYPES_INCLUDE})
    message(STATUS "setting LIBVIDEOIO-TYPES_INCLUDE to \"${CMAKE_CURRENT_SOURCE_DIR}/include\"")
    unset(LIBVIDEOIO-TYPES_INCLUDE)
    set(LIBVIDEOIO-TYPES_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include PARENT_SCOPE)
    #message(STATUS "1 \${LIBVIDEOIO-TYPES_INCLUDE1} \"${LIBVIDEOIO-TYPES_INCLUDE1}\"")
endif()
#message(STATUS "2 \${LIBVIDEOIO-TYPES_INCLUDE} \"${LIBVIDEOIO-TYPES_INCLUDE}\"")
#if( NOT FIPS_IMPORT )
#	if( FIPS_UNITTESTS )
#	  fips_add_subdirectory( test/unit/ )
#	endif()
#
#	fips_finish()
#endif()
