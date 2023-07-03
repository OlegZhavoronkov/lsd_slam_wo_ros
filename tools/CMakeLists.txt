
## TODO.  Strip GUI elements from this repo.
add_executable(LSD LSD.cpp
../lib/App/InputThread.cpp)
#fips_begin_app(LSD cmdline)
#  include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

#  fips_files(  )

target_link_libraries(LSD g3log g3logger lsdslam videoio )
#  fips_deps(  )
#fips_end_app()
