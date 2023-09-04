#include <opencv2/calib3d/calib3d.hpp>
#include "libvideoio/Undistorter.h"

namespace libvideoio
{

ImageCropper::ImageCropper( int width, int height, int offsetX , int offsetY ,
              const std::shared_ptr<Undistorter> & wrap)
              : Undistorter(wrap),
               _offsetX(offsetX), 
               _offsetY(offsetY),
               _width(width),      
               _height(height)
{
    if(wrap==nullptr)
    {
        throw std::runtime_error("wrapped undistorter should not be null");
    }
    auto initialK=wrap->getK();
    auto inputSize=ImageCropper::inputImageSize().cvSize();
    auto outputSize=ImageCropper::outputImageSize().cvSize();
    _K = cv::getOptimalNewCameraMatrix(initialK, cv::Mat::zeros(cv::Size(1,4),CV_64F),
            inputSize,
            0,              // 0 == all pixels in un-distorted image are valid
            outputSize, nullptr, false);
}

void ImageCropper::undistort(const cv::Mat &image, cv::OutputArray result) const
{
    cv::Mat intermediate(image);
    if( _wrapped ) 
    {
        _wrapped->undistort( image, intermediate );
    }
    if(image.cols> _width && image.rows>_height)
    {
        cv::Mat roi( intermediate, cv::Rect( _offsetX, _offsetY, _width, _height ) );
        LOG(WARNING) << "Cropping to " << _width << " x " << _height;
    // cv::imshow("roi",roi);
    // cv::waitKey(10);
        result.assign( roi );
    }
    else
    {
        cv::Mat copy;
        cv::resize( intermediate,copy ,cv::Size(_width,_height),0.0,0.0,cv::INTER_LINEAR);
        LOG(WARNING) << "Cropping to " << _width << " x " << _height;
    // cv::imshow("roi",roi);
    // cv::waitKey(10);
        result.assign( copy );
    }
}

  /**
   * Returns the intrinsic parameter matrix of the undistorted images.
   */
  //const cv::Mat getK() const;

const cv::Mat ImageCropper::getK() const 
{
    if( _wrapped )
    {
        return _K;
    }
    return cv::Mat::eye(3,3, CV_32F );
}


  /**
   * Returns the intrinsic parameter matrix of the original images,
   */
const cv::Mat ImageCropper::getOriginalK() const 
{
    return getK(); 
}

ImageSize ImageCropper::outputImageSize( void ) const
{
    return ImageSize( _width, _height ); 
}

ImageSize ImageCropper::inputImageSize( void ) const
{ 
    if( _wrapped ) 
    {
        return _wrapped->outputImageSize();
    }
    return ImageSize( _width, _height ); 
}

  /**
   * Returns the width of the input images in pixels.
   */
int ImageCropper::getInputWidth() const 
{
    if( _wrapped ) 
    {
        return _wrapped->getInputWidth();
    }
    return _width;
}

  /**
   * Returns the height of the input images in pixels.
   */
int ImageCropper::getInputHeight() const 
{
    if( _wrapped ) 
    {
        return _wrapped->getInputHeight();
    }
    return _width;
}


  /**
   * Returns if the undistorter was initialized successfully.
   */
bool ImageCropper::isValid() const 
{
    return true; 
}


}