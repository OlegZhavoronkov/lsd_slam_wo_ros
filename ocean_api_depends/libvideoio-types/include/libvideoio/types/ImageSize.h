#pragma once

#include <opencv2/core.hpp>

namespace libvideoio {

struct ImageSize {
  ImageSize( void ) : width(0), height(0) {;}

  ImageSize( int w, int h )
    : width(w), height(h)
  {;}

  ImageSize( const cv::Size &sz )
    : width(sz.width), height(sz.height)
  {;}


  long int area( void ) const { return width*height; }
  float aspectRatio( void ) const { return float(width)/float(height); }

  cv::Size cvSize( void ) const { return cv::Size( width, height); }
  cv::Size operator()( void ) const { return cv::Size( width, height); }

  int width, height;
};

// // Specialization of ImageSize with additional input checking
// struct SlamImageSize : public ImageSize {
//   SlamImageSize( void ) : ImageSize() {;}
//
//   SlamImageSize( int w, int h )
//     : ImageSize( w, h )
//   {
//     CHECK(w%16 == 0 && h%16 == 0) << "SLAM image dimensions must be multiples of 16! Please crop your images / video accordingly.";
//     CHECK(     w!=0 && h!=0 ) << "Height or width set to zero!";
//   }
//
//   SlamImageSize &operator=(const ImageSize &other) { width = other.width; height = other.height; return *this; }
// };

}
