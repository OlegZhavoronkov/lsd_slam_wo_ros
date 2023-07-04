#find_package(Boost COMPONENTS thread filesystem system REQUIRED)

# Set source files
#set(logger_SOURCE_FILES
#  ${CMAKE_CURRENT_SOURCE_DIR}/LogReader.cpp
#  ${CMAKE_CURRENT_SOURCE_DIR}/LogWriter.cpp
#)
#add_library(binlogger SHARED "")
#target_sources(
#    binlogger
#    PRIVATE ${logger_SOURCE_FILES}
#)
#target_link_libraries(binlogger PUBLIC
#${Boost_LIBRARIES} ${ZLIB_LIBRARIES} ${OpenCV_LIBS}
#g3logger activeobject
#)
# build shared library.
#fips_begin_module(binlogger)
#  fips_files(
#    LogReader.cpp
#    LogWriter.cpp
#  )
#  fips_libs( )
#  fips_deps( g3logger activeobject )
#fips_end_module()
