
add_library(activeobject SHARED active.cpp active_idle.cpp)
message(STATUS "\${LIBACTIVE_OBJECT_INCLUDE_DIRS_LOCAL} \"${LIBACTIVE_OBJECT_INCLUDE_DIRS_LOCAL}\"")
target_include_directories(
    activeobject
    PUBLIC
  ${Boost_INCLUDE_DIR}
  ${LIBACTIVE_OBJECT_INCLUDE_DIRS_LOCAL}
)

target_link_libraries(activeobject PRIVATE ${Boost_LIBRARIES})
# build shared library.
#fips_begin_module(activeobject)
#  fips_files(active.cpp active_idle.cpp)
#  fips_libs( ${Boost_LIBRARIES} )
#fips_end_module()
