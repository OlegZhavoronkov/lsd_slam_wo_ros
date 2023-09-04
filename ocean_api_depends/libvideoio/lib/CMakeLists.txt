# Set source files

set(VIDEOIO_SOURCE_FILES)
    list(APPEND VIDEOIO_SOURCE_FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/Display.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ImageOutput.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ImageSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/VideoOutput.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/undistorter/OpenCVUndistorter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/undistorter/OpenCVUndistorterFactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/undistorter/PhotoscanXMLUndistorterFactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/undistorter/PTAMUndistorter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/undistorter/PTAMUndistorterFactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/undistorter/ROSUndistorterFactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/undistorter/UndistorterFactory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/undistorter/Undistorter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}//util/FileUtils.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}//util/old/UndistorterLogger.cpp
    #${CMAKE_CURRENT_SOURCE_DIR}//util/old/UndistorterPTAM.cpp    
)

#file(GLOB VIDEOIO_SOURCE_FILES *.cpp
#        util/*.cpp
#        undistorter/*.cpp )

# build shared library.
add_library(videoio SHARED "")
#fips_begin_module( videoio )
target_sources(videoio PRIVATE ${VIDEOIO_SOURCE_FILES})
  #fips_files( ${VIDEOIO_SOURCE_FILES} )

set(PUBLIC_LIBVIDEIO_LIBS
        ${Boost_LIBRARIES} 
        ${OpenCV_LIBS}
        ${ZLIB_LIBRARIES} 
        #${TINYXML2_LIBRARIES}
        g3logger
        activeobject
        binlogger)

if(${TINYXML2_FOUND} STREQUAL "FALSE")
  message(STATUS "2 \${TINYXML2_FOUND} \"${TINYXML2_FOUND}\" not found")
else()
  list(APPEND PUBLIC_LIBVIDEIO_LIBS ${TINYXML2_LIBRARIES})
  message(STATUS "2 \${TINYXML2_FOUND} \"${TINYXML2_FOUND}\" found")
  target_compile_definitions(videoio PUBLIC WITH_TINYXML)
endif()

target_link_libraries(videoio PUBLIC
                    ${PUBLIC_LIBVIDEIO_LIBS}
            )
  #fips_deps( g3logger )
message(STATUS "videoio \${LIBVIDEOIO-TYPES_INCLUDE} \"${LIBVIDEOIO-TYPES_INCLUDE}\"")
target_include_directories(videoio PUBLIC ${LIBVIDEOIO-TYPES_INCLUDE} ${VIDEOIO_INCLUDE_DIRS}
                                    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
#  include_directories(
#    #${LIBVIDEOIO-TYPES_INCLUDE}
#    
#    
#  )
if(NOT "X${YAML_CPP_INCLUDE_DIR}" STREQUAL "X")
    target_compile_definitions(videoio PUBLIC HAS_YAML_CPP)
endif()
#fips_end_module()
