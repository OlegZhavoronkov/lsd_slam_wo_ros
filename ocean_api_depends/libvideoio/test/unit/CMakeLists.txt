include_directories( ${CMAKE_SOURCE_DIR}/lib )

file( GLOB UNIT_TEST_SRCS *_test.cpp test_*.cpp )

gtest_begin(videoio NO_TEMPLATE)
    fips_files(
      ${UNIT_TEST_SRCS}
      main.cpp
    )
    fips_deps(videoio)

    include_directories( ${TEST_DATA_DIR} )
gtest_end()
