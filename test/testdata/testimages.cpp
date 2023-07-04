#include "testimages.h"

#include "base64_impl.h"
#include <opencv2/core.hpp>

std::vector<BYTE> TestImage( int i ) {
  std::vector<BYTE> out = base64_decode( TestImages[i] );
  return out;
}

cv::Mat TestImageCvMat( int i ) {
  std::vector<BYTE> b = base64_decode( TestImages[i] );

  // Somewhat awkward but this keeps memory management simple for now
  cv::Mat bmat( cv::Size( TestImageWidth, TestImageHeight ), CV_8UC1, b.data() );
  cv::Mat out;

  bmat.copyTo( out );

  return out;
}
