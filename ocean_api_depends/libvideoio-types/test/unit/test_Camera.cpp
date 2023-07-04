
#include <iostream>

#include <gtest/gtest.h>

#include "libvideoio/types/Camera.h"

using namespace libvideoio;
using namespace std;

namespace {

void CheckCamera( const Camera &a, float fx, float fy, float cx, float cy ) {

  ASSERT_FLOAT_EQ( a.fx, fx );
  ASSERT_FLOAT_EQ( a.fy, fy );
  ASSERT_FLOAT_EQ( a.cx, cx );
  ASSERT_FLOAT_EQ( a.cy, cy );

  const float fxi = 1.0/fx, fyi = 1.0/fy,
              cxi = -cx/fx, cyi = -cy/fy;

  ASSERT_FLOAT_EQ( a.fxi, fxi );
  ASSERT_FLOAT_EQ( a.fyi, fyi );
  ASSERT_FLOAT_EQ( a.cxi, cxi );
  ASSERT_FLOAT_EQ( a.cyi, cyi );

  // Eigen's paren operator is column-major...
  std::array<float,9> k = {fx, 0.0, 0.0, 0.0, fy, 0.0, cx, cy, 1.0 };
  for( int i = 0; i < 8; ++i ) {
    ASSERT_FLOAT_EQ( k[i], a.K(i) ) << "for i = " << i << endl << a.K;
  }

  std::array<float,9> ki = {fxi, 0.0, 0.0, 0.0, fyi, 0.0, cxi, cyi, 1.0 };
  for( int i = 0; i < 8; ++i ) {
    ASSERT_FLOAT_EQ( a.Kinv(i), ki[i] ) << "for i = " << i;
  }

}

void CompareCameras( const Camera &a, const Camera &b ) {
  ASSERT_FLOAT_EQ( a.fx, b.fx );
  ASSERT_FLOAT_EQ( a.fy, b.fy );
  ASSERT_FLOAT_EQ( a.cx, b.cx );
  ASSERT_FLOAT_EQ( a.cy, b.cy );

  ASSERT_FLOAT_EQ( a.fxi, b.fxi );
  ASSERT_FLOAT_EQ( a.fyi, b.fyi );
  ASSERT_FLOAT_EQ( a.cxi, b.cxi );
  ASSERT_FLOAT_EQ( a.cyi, b.cyi );
}


TEST( Camera, Constructor ) {
  const float fx = 500, fy = 500, cx = 320, cy = 240;
  Camera cam( fx, fy, cx, cy );

  CheckCamera( cam, fx, fy, cx, cy );
}

TEST( Camera, CopyConstructor ) {
  const float fx = 500, fy = 500, cx = 320, cy = 240;
  Camera camA( fx, fy, cx, cy );

  Camera camB( camA );

  CompareCameras( camA, camB );
}

TEST( Camera, SetConstructor ) {
  const float fx = 500, fy = 500, cx = 320, cy = 240;
  Camera camA( fx, fy, cx, cy );

  Camera camB = camA;

  CompareCameras( camA, camB );
}

}
