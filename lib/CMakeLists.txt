
set(GUI_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
)

# Set source files
set(lsdslam_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/App/InputThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/Frame.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/ImageSet.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/FramePoseStruct.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/FrameMemory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/KeyFrame.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DepthEstimation/DepthMap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DepthEstimation/DepthMapDebugImages.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/Configuration.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/Configuration.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/globalFuncs.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/SophusUtil.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/settings.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/Sim3Tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/Relocalizer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/SE3Tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/SE3Tracker_SSE.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/SE3Tracker_NEON.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/FabMap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/KeyFrameGraph.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/TrackableKeyFrameSearch.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/OptimizationThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/MappingThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/TrackingThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/ConstraintSearchThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem.cpp
  ${GUI_SRCS}
)

add_library(lsdslam SHARED "")
target_sources(lsdslam PUBLIC ${lsdslam_SOURCE_FILES})
message(STATUS "lsdslam \"\${LIBVIDEOIO-TYPES_INCLUDE} \"${LIBVIDEOIO-TYPES_INCLUDE}\"")
#fips_begin_module( lsdslam )
if(NOT LIBVIDEOIO-TYPES_INCLUDE STREQUAL "${INITIAL_LIBVIDEOIO-TYPES_INCLUDE}")
    target_include_directories(lsdslam PUBLIC ${LIBVIDEOIO-TYPES_INCLUDE})
endif()
#  fips_files( ${lsdslam_SOURCE_FILES} )
message(STATUS "\${G2O_LIBRARIES} \"${G2O_LIBRARIES}\"")
  # build shared library.
target_link_libraries(lsdslam
    PRIVATE
  #fips_libs(
    ${FABMAP_LIB}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${CSPARSE_LIBRARY} 
    #)

  #fips_deps( 
    glog::glog 
    activeobject 
    ${G2O_LIBRARIES} 
    )

#fips_end_module()
