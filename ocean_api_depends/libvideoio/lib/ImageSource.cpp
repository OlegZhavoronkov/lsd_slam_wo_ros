#include <g3log/g3log.hpp>

#include "libvideoio/ImageSource.h"

namespace libvideoio {

  int ImageSource::getImage( int i, cv::Mat &mat ) {
    int ret = getRawImage(i,mat);
    if( _outputType <0 || mat.type() == _outputType ) return ret;

    //auto inChannels  = ((mat.type()  & ~CV_MAT_DEPTH_MASK) >> CV_CN_SHIFT) + 1;
    auto outChannels = ((_outputType & ~CV_MAT_DEPTH_MASK) >> CV_CN_SHIFT) + 1;

    if( outChannels == 3 ) {
      CHECK( cvtToRGB() >= 0 ) << "No conversion to RGB specified by ImageSource";
      cv::Mat tmp;
      cv::cvtColor( mat, tmp, cvtToRGB() );
      tmp.convertTo( mat, _outputType );
    } else if( outChannels == 1 ) {
      CHECK( cvtToGray() >= 0 ) << "No conversion to gray specific by ImageSource";
      cv::Mat tmp;
      cv::cvtColor( mat, tmp, cvtToGray() );
      tmp.convertTo( mat, _outputType );
    } else {
      LOG(FATAL) << "Unable to figure out how to convert to " << outChannels << " channels";
    }

    return ret;
  }


}
