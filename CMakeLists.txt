
## Error message if not run in fips
#if (NOT FIPS_CONFIG)
#  message(FATAL_ERROR "As of December 2017, I'm using \"fips\" as a package manager.   The \"cmake\" branch contains the working head for the cmake-based build.  See README.md for more information.")
#endif()

#if (NOT FIPS_IMPORT)
cmake_minimum_required(VERSION 3.5.0)

#  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
#  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")
project(lsd_slam)
#fips_setup(PROJECT lsd_slam)
#endif()

option(BUILD_GUI "Build Pangolin-based GUI" OFF)
set(EMPTY_EXPLICIT_G2O_PATH "EXPLICIT_G2O_PATH_NOT_DEFINED")
set(EXPLICIT_G2O_PATH "${EMPTY_EXPLICIT_G2O_PATH}" CACHE PATH "path to g2o-provided cmake")

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)




## I believe the fipsified version of g2o will handle this...
# ## I haven't packaged suitesparse yet...

find_package(SuiteSparse REQUIRED)
find_package(Ceres)
#find_package( TinyXML2  )
#message(STATUS "\${TINYXML2_FOUND} \"${TINYXML2_FOUND}\"")
find_package(OpenGL)
find_package(fmt REQUIRED)
#find_package(glog REQUIRED)
# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )


##==== Check and define the architecture
## Architecture-specific optimizations
if( NOT CUDA_VERSION_EXACT )
    set( CUDA_VERSION_EXACT "11.6" CACHE STRING  "Use CUDA")
    # set( CUDA_VERSION_EXACT "11.8" CACHE STRING  "Use CUDA")
endif()

if( NOT CUDA_VERSION)
    set(CUDA_VERSION ${CUDA_VERSION_EXACT})
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")

message(STATUS "CMAKE_CXX_STANDARD: ${CMAKE_CXX_STANDARD}")
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
	## For compilation on Jetson TX1
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a") # -DENABLE_NEON" )

elseif( ${ARCHITECTURE} STREQUAL "x86_64" )
    message(STATUS "\${CMAKE_BUILD_TYPE} \"${CMAKE_BUILD_TYPE}\"")
  if( ${CMAKE_BUILD_TYPE} MATCHES Release )
    # -DENABLE_SSE enables Jakob's original SSE optimizations in his code,
  	# it has nothing to do with compiler behavior
    message( STATUS " --> Using SSE" )
    ## Enable SSE on release only...
    #set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.1 -msse3 -msse2 -msse -march=native -DENABLE_SSE" )
    set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.1 -msse3 -msse2 -msse -march=native" )
  elseif(${CMAKE_BUILD_TYPE} MATCHES RelWithDebInfo)
        # -DENABLE_SSE enables Jakob's original SSE optimizations in his code,
          # it has nothing to do with compiler behavior
        message( STATUS " --> Using SSE" )
        ## Enable SSE on release only...
        #set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.1 -msse3 -msse2 -msse -march=native -DENABLE_SSE" )
        set( CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -msse4.1 -msse3 -msse2 -msse -march=native" )
   else() 
        message( STATUS " --> Not enabling SSE on x86_64 architecture for Debug build")
  endif()
endif()
if( CUDA_VERSION )
	find_package( CUDA REQUIRED 
    #${CUDA_VERSION} 
    )
  SET( CUDA_USE_STATIC_CUDA_RUNTIME OFF )
endif()

set( OPENCV_COMPONENTS core highgui imgproc video calib3d )
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

if( OpenCV_VERSION_MAJOR GREATER_EQUAL "3" )
  message( STATUS "Using OpenCV 3.x")
  add_definitions( -DOPENCV3 )
else()
  message( STATUS "Using OpenCV 2.x")
  add_definitions( -DOPENCV2 )
endif()


## C++ flags applied to all builds
#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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

include_directories( ${LSDSLAM_INCLUDE_DIRS} )



if(NOT EXPLICIT_G2O_PATH STREQUAL EMPTY_EXPLICIT_G2O_PATH)
    #find_package(Eigen3)
    #find_package(OpenGL)
    message(STATUS "Starting search for g2o EIGEN3_INCLUDE_DIR \"${EIGEN3_INCLUDE_DIR}\" EIGEN3_FOUND \"${EIGEN3_FOUND}\" OpenGL_FOUND \"${OpenGL_FOUND}\" OPENGL_FOUND \"${OPENGL_FOUND}\"")
    if(${EIGEN3_FOUND}   )
        set(REAL_G2O_CONFIG_PATH "${EXPLICIT_G2O_PATH}/g2oTargets.cmake")
        find_package(Eigen3 REQUIRED)
        #find_file(REAL_G2O_CONFIG_PATH "g2oTargets.cmake" PATHS ${EXPLICIT_G2O_PATH})
        include("${REAL_G2O_CONFIG_PATH}")
        #find_package(g2o REQUIRED CONFIG PATHS "${EXPLICIT_G2O_PATH}" NO_DEFAULT_PATH)
        list(APPEND g2o_libraries g2o)
        message(STATUS "g2o_libraries \"${g2o_libraries}\"")
        set(NATIVE_G2O_CMAKE_USED "TRUE")
        #return()
    else()
        message(STATUS  "either Eigen or OpenGL which are dependencies of g2o not found")
        #return()
    endif()
endif()
set(G2O_LIBRARIES)
if(${NATIVE_G2O_CMAKE_USED})
    list(APPEND G2O_LIBRARIES 
            g2o::core 
            g2o::stuff 
            g2o::solver_csparse 
            g2o::csparse_extension 
            g2o::types_sim3 
            g2o::types_sba)
else()
    list(APPEND G2O_LIBRARIES 
            g2o_core 
            g2o_stuff 
            g2o_solver_csparse 
            g2o_csparse_extension 
            g2o_types_sim3 
            g2o_types_sba)
endif()
set(INITIAL_LIBVIDEOIO-TYPES_INCLUDE "LIBVIDEOIO-TYPES_INCLUDE_NOT_SET")

if(NOT LIBVIDEOIO-TYPES_INCLUDE)
    set(LIBVIDEOIO-TYPES_INCLUDE ${INITIAL_LIBVIDEOIO-TYPES_INCLUDE} )
endif()
add_subdirectory(ocean_api_depends/g3logg/)
add_subdirectory(ocean_api_depends/libg3logger/)
add_subdirectory(ocean_api_depends/libvideoio-types/)

add_subdirectory(ocean_api_depends/libbinlogger/)
add_subdirectory(ocean_api_depends/libvideoio/)
add_subdirectory(ocean_api_depends/libactiveobject/)
message(STATUS "\"root LIBVIDEOIO-TYPES_INCLUDE\" \"${LIBVIDEOIO-TYPES_INCLUDE}\"")
#if(g2o_libraries)
#    set(G2O_LIBRARIES ${g2o_libraries})
#endif()
add_subdirectory( lib/ )

## Provide option to not build tools, default to yes unless FIP_IMPORT
option( LSDSLAM_BUILD_TOOLS ON )
#if( NOT FIPS_IMPORT )
#  set( LSDSLAM_BUILD_TOOLS OFF )
#endif()
#if( LSDSLAM_BUILD_TOOLS )
  #fips_add_subdirectory( tools/ )
  add_subdirectory( tools/ )
#endif()

#if( NOT FIPS_IMPORT )
#
#  if( FIPS_UNITTESTS )
#    ## Build a local copy of gtest (as per the GTest FAQ)
#    fips_add_subdirectory( test/unit/ )
#  endif()
#
#  fips_finish()
#endif()
