cmake_minimum_required(VERSION 3.5)
#if( NOT FIPS_IMPORT )
#	get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
#	include("${FIPS_ROOT_DIR}/cmake/fips.cmake")
#
#	fips_setup(PROJECT libvideoio)
#endif()
project(libvideoio LANGUAGES C CXX)

find_package( ZLIB REQUIRED )
find_package( Threads )
find_package( Boost COMPONENTS filesystem system REQUIRED)
find_package( Eigen3 REQUIRED )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package( TinyXML2 REQUIRED )
find_package( yaml-cpp )

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a") # -DENABLE_NEON" )
elseif( ${ARCHITECTURE} STREQUAL "x86_64" )
  # -DENABLE_SSE enables Jakob's original SSE optimizations in his code,
	# it has nothing to do with compiler behavior
  set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.1 -msse3 -msse2 -msse -march=native -DENABLE_SSE" )
endif()

## C++ flags applied to all builds
## squelch int-in-bool-context warnings from eigen
##"-Wno-int-in-bool-context")
#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
## Configure OpenCV ##
set( OPENCV_COMPONENTS core highgui imgproc video calib3d )
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

if( OpenCV_VERSION_MAJOR GREATER_EQUAL "3" )
  message( STATUS "Using OpenCV 3.x")
  add_definitions( -DOPENCV3 )
else()
  message( STATUS "Using OpenCV 2.x")
  add_definitions( -DOPENCV2 )
endif()

## Need this workaround for CUDA 8.0
#find_package( CUDA OPTIONAL 8.0 )
if( CUDA_VERSION )
	message("Found CUDA version ${CUDA_VERSION}")
	if(${CUDA_VERSION} VERSION_EQUAL "8.0")
		message(STATUS "Disabling CUDA_USE_STATIC_CUDA_RUNTIME in libvideoio")
    SET( CUDA_USE_STATIC_CUDA_RUNTIME OFF CACHE BOOL "" FORCE )
		message(STATUS ${CUDA_USE_STATIC_CUDA_RUNTIME} )
	endif()

	find_package( CUDA REQUIRED ${CUDA_VERSION} )
endif()


## Set global include paths
set( VIDEOIO_INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/third_party
  ${OpenCV_INCLUDE_DIRS}
	${TINYXML2_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
  #${YAML_CPP_INCLUDE_DIR}
   )

if(${YAML_CPP_FOUND})
    list(APPEND VIDEOIO_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})
else()
    message(STATUS "\${YAML_CPP_FOUND} not found: \"${YAML_CPP_FOUND}\" \${YAML_CPP_INCLUDE_DIR} \"${YAML_CPP_INCLUDE_DIR}\"")
endif()
include_directories( ${VIDEOIO_INCLUDE_DIRS} )

add_subdirectory("lib/")


## Provide option to not build tools, default to yes unless FIP_IMPORT
option( LIBVIDEOIO_BUILD_TOOLS OFF )
if( NOT FIPS_IMPORT )
  set( LIBVIDEOIO_BUILD_TOOLS OFF )
endif()
if( LIBVIDEOIO_BUILD_TOOLS )
#  fips_add_subdirectory( tools/ )
endif()


#if( NOT FIPS_IMPORT )
#	set( TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/data" )
#	add_definitions( -DTEST_DATA_DIR="${TEST_DATA_DIR}" )
#
#	if( FIPS_UNITTESTS )
#		message("** Will run unit tests")
#	  fips_add_subdirectory( test/unit/ )
#	endif()
#
#	fips_finish()
#endif()
