
#include "libvideoio/Undistorter.h"

#include <tinyxml2.h>

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace libvideoio
{

  OpenCVUndistorter::OpenCVUndistorter( const cv::Mat &k,
                                        const cv::Mat &distCoeff,
                                        const ImageSize &origSize,
                                        const std::shared_ptr<Undistorter> & wrap)
    : Undistorter(wrap),
      _originalK( k ),
      _distCoeffs( distCoeff ),
      _baseline(0.0, 0.0, 0.0),
      _inputSize( origSize ),
      _outputSize( origSize ),
      _valid( true )
{

  _K = cv::getOptimalNewCameraMatrix(_originalK, _distCoeffs,
                _inputSize(),
                0,              // 0 == all pixels in un-distorted image are valid
                _outputSize(), nullptr, false);

  cv::initUndistortRectifyMap(_originalK, _distCoeffs, cv::Mat(), _K,
                      _outputSize(), CV_16SC2, _map1, _map2);

  // Need to check on reason for this
  // _originalK.at<double>(0, 0) /= _inputSize.width;
  // _originalK.at<double>(0, 2) /= _inputSize.width;
  // _originalK.at<double>(1, 1) /= _inputSize.height;
  // _originalK.at<double>(1, 2) /= _inputSize.height;

}

OpenCVUndistorter::OpenCVUndistorter( const cv::Mat &origK,
                                      const cv::Mat &projection,
                                      const cv::Mat &rectification,
                                      const cv::Mat &distCoeff,
                                      const ImageSize &origSize,
                                      const std::shared_ptr<Undistorter> &wrap )
    : Undistorter(wrap),
      _originalK( origK ),
      _K( cv::Mat( projection, cv::Rect(0,0,3,3) )  ),
      _distCoeffs( distCoeff ),
      _baseline(0.0, 0.0, 0.0),
      _inputSize( origSize ),
      _outputSize( origSize ),
      _valid( true )
{

  cv::initUndistortRectifyMap(_originalK, _distCoeffs, rectification, _K,
                      _outputSize(), CV_16SC2, _map1, _map2);

  // Calculate baseline
  _baseline[0] = -projection.at<double>(0,3) / projection.at<double>(0,0);
  _baseline[1] = _baseline[2] = 0.0;
}



OpenCVUndistorter::~OpenCVUndistorter()
{}

void OpenCVUndistorter::undistort(const cv::Mat& image, cv::OutputArray result) const
{
  cv::Mat intermediate(image);
  if( _wrapped ) {
    _wrapped->undistort( image, intermediate );
  }

	 cv::remap(intermediate, result, _map1, _map2, cv::INTER_LINEAR);

  if( false ) {
     cv::Mat inputScaled;
     cv::resize( image, inputScaled, cv::Size( image.size().width/2, image.size().height/2 ) );
     cv::imshow( name() + " original", inputScaled );


     cv::Mat scaled;
     cv::resize( result, scaled, cv::Size( result.size().width/2, result.size().height/2 ) );
     cv::imshow( name() + " rectified", scaled );
   }

}


}
