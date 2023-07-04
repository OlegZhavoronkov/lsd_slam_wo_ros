

gtest_begin( lsdslam )

  set( TESTIMAGES_SRCS ../testdata/testimages.cpp
                      ../testdata/testimages_data.cpp )

  fips_include_directories(../testdata)

    fips_files(
      test_FrameData.cpp
      test_Frame.cpp
      ../testdata/test_testimages.cpp
      ${TESTIMAGES_SRCS}
    )

    fips_deps( lsdslam videoio )
gtest_end()
