include_directories( ${CMAKE_SOURCE_DIR}/lib )

file( GLOB UNIT_TEST_SRCS *_test.cpp test_*.cpp )

gtest_begin(videoio_types)
    fips_files(
      ${UNIT_TEST_SRCS}
    )
gtest_end()
