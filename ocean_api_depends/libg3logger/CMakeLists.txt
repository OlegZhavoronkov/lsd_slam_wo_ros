cmake_minimum_required(VERSION 3.5)
#if( NOT FIPS_IMPORT )
#	get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
#	include("${FIPS_ROOT_DIR}/cmake/fips.cmake")
project(libg3logger LANGUAGES C CXX)
#	fips_setup(PROJECT libg3logger)
#endif()
#include_directories(include/)
#fips_include_directories( include/ )

#add_subdirectory("lib/")

#if( NOT FIPS_IMPORT )
#	#fips_add_subdirectory("tools/")
#
#	if( FIPS_UNITTESTS )
#	  fips_add_subdirectory( test/unit/ )
#	endif()
#
#	fips_finish()
#endif()
add_library(g3logger SHARED "")
target_include_directories(g3logger PUBLIC include)
target_sources(g3logger 
                INTERFACE 
                        ${CMAKE_CURRENT_SOURCE_DIR}/include/libg3logger/g3logger.h
                        ${CMAKE_CURRENT_SOURCE_DIR}/include/libg3logger/ColorStderrSink.h
                PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib/g3logger.cpp)
target_link_libraries(g3logger PUBLIC g3log)
