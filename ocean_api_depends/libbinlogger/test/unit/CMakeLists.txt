FILE(GLOB UNIT_TEST_SRCS *.cpp )

gtest_begin(binlogger)
    fips_files(
            ${UNIT_TEST_SRCS}
    )
    fips_deps(g3logger activeobject)
gtest_end()

