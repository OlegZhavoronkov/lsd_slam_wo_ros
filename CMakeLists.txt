
## Error message if not run in fips
if (NOT FIPS_CONFIG)
  message(FATAL_ERROR "As of December 2017, I'm using \"fips\" as a package manager.   The \"cmake\" branch contains the working head for the cmake-based build.  See README.md for more information.")
endif()

if (NOT FIPS_IMPORT)
  cmake_minimum_required(VERSION 3.5.0)

  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

  fips_setup(PROJECT lsd_slam)
endif()

option(BUILD_GUI "Build Pangolin-based GUI" OFF)

find_package(Boost COMPONENTS thread filesystem system REQUIRED)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## I believe the fipsified version of g2o will handle this...
# ## I haven't packaged suitesparse yet...
find_package(SuiteSparse REQUIRED)
find_package( TinyXML2 REQUIRED )

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )


##==== Check and define the architecture
## Architecture-specific optimizations
option( CUDA_VERSION "Use CUDA" "")

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
	## For compilation on Jetson TX1
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a") # -DENABLE_NEON" )

elseif( ${ARCHITECTURE} STREQUAL "x86_64" )
  if( ${CMAKE_BUILD_TYPE} MATCHES RELEASE )
    # -DENABLE_SSE enables Jakob's original SSE optimizations in his code,
  	# it has nothing to do with compiler behavior
    message( STATUS " --> Using SSE" )
    ## Enable SSE on release only...
    set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.1 -msse3 -msse2 -msse -march=native -DENABLE_SSE" )
  else()
    message( STATUS " --> Not enabling SSE on x86_64 architecture for Debug build")
  endif()
endif()

set( OPENCV_COMPONENTS core highgui imgproc video calib3d )
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

if( OpenCV_VERSION_MAJOR EQUAL "3" )
  message( STATUS "Using OpenCV 3.x")
  add_definitions( -DOPENCV3 )
else()
  message( STATUS "Using OpenCV 2.x")
  add_definitions( -DOPENCV2 )
endif()

if( CUDA_VERSION )
	find_package( CUDA REQUIRED ${CUDA_VERSION} )
  SET( CUDA_USE_STATIC_CUDA_RUNTIME OFF )
endif()

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set( LSDSLAM_INCLUDE_DIRS
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/lib/lsd_core
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${Pangolin_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
	${PROJECT_SOURCE_DIR}/thirdparty/Sophus )

if(APPLE)
	set( CMAKE_MACOSX_RPATH OFF )
endif()

fips_include_directories( ${LSDSLAM_INCLUDE_DIRS} )

fips_add_subdirectory( lib/ )

## Provide option to not build tools, default to yes unless FIP_IMPORT
option( LSDSLAM_BUILD_TOOLS ON )
if( NOT FIPS_IMPORT )
  set( LSDSLAM_BUILD_TOOLS OFF )
endif()
if( LSDSLAM_BUILD_TOOLS )
  fips_add_subdirectory( tools/ )
endif()

if( NOT FIPS_IMPORT )

  if( FIPS_UNITTESTS )
    ## Build a local copy of gtest (as per the GTest FAQ)
    fips_add_subdirectory( test/unit/ )
  endif()

  fips_finish()
endif()
