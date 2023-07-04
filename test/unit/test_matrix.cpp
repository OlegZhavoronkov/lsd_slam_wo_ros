
#include <gtest/gtest.h>

TEST( matrix, constructor )
{
  const size_t xsize = 4;
  const size_t ysize = 3;

  Matrix<float,xsize,ysize> foo;

  // X dimension
  ASSERT_EQ( foo.size(), xsize );

  // Y dimension
  ASSERT_EQ( foo[0].size(), ysize);
}
