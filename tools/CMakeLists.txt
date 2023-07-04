
## TODO.  Strip GUI elements from this repo.

fips_begin_app(LSD cmdline)
  include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

  fips_files( LSD.cpp
              ../lib/App/InputThread.cpp )

  fips_deps( g3log g3logger lsdslam videoio )
fips_end_app()
