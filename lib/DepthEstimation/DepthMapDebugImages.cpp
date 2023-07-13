

#include <opencv2/imgproc.hpp>

#include "DepthMapDebugImages.h"
#include "IOWrapper/ImageDisplay.h"
#include "util/globalFuncs.h"


namespace lsd_slam {

  using namespace cv;

  DepthMapDebugImages::DepthMapDebugImages( const ImageSize &imgSize )
  : _imageSize( imgSize ),
  _debugImageHypothesisHandling( _imageSize.cvSize(), CV_8UC3 ),
  _debugImageHypothesisPropagation( _imageSize.cvSize(), CV_8UC3 ),
  _debugImageStereoLines( _imageSize.cvSize(), CV_8UC3 ),
  _debugImageDepth( _imageSize.cvSize(), CV_8UC3 )
  {;}

  DepthMapDebugImages::~DepthMapDebugImages()
  {;}

/*
  void DepthMapDebugImages::plotUpdateKeyFrame( const Frame::SharedPtr &activeKeyFrame,
    const Frame::SharedPtr &oldestReferenceFrame,
    const Frame::SharedPtr &newestReferenceFrame )
  {
    cv::Mat keyFrameImage(_imageSize.height, _imageSize.width, CV_32F, const_cast<float*>(activeKeyFrame->image(0)));
    keyFrameImage.convertTo(_debugImageHypothesisHandling, CV_8UC1);
    cv::cvtColor(_debugImageHypothesisHandling, _debugImageHypothesisHandling, cv::COLOR_GRAY2RGB);

    cv::Mat oldest_refImage(_imageSize.height, _imageSize.width, CV_32F, const_cast<float*>(oldestReferenceFrame->image(0)));
    cv::Mat newest_refImage(_imageSize.height, _imageSize.width, CV_32F, const_cast<float*>(newestReferenceFrame->image(0)));
    cv::Mat rfimg = 0.5f*oldest_refImage + 0.5f*newest_refImage;
    rfimg.convertTo(_debugImageStereoLines, CV_8UC1);
    cv::cvtColor(_debugImageStereoLines, _debugImageStereoLines, cv::COLOR_GRAY2RGB);
  }*/

  void DepthMapDebugImages::setHypotesisAndLineImages(Frame::SharedPtr activeKeyFrame,const cv::Mat& hypotesis,const cv::Mat& lineImage)
  {
    std::scoped_lock lock(_lock);
    
    if(activeKeyFrame != nullptr)
    {
        std::unique_ptr<float[]> buff(new float[activeKeyFrame->imgSize(0).area() ]);
        memcpy(buff.get(),activeKeyFrame->image(0),activeKeyFrame->imgSize(0).area()*sizeof(float));
        {
            cv::Mat keyFrameImage(_imageSize.height, _imageSize.width, CV_32F,buff.get());
            cv::Mat tempKeyFrame;
            keyFrameImage.convertTo(tempKeyFrame, CV_8UC1);
            cv::cvtColor(tempKeyFrame, tempKeyFrame, cv::COLOR_GRAY2RGB);
            _debugImageStereoLines=0.7*lineImage+tempKeyFrame;
        }
        
    }
    else
    {
        lineImage.copyTo(_debugImageStereoLines);
    }
    
    hypotesis.copyTo(_debugImageHypothesisHandling);
    
  }

  void DepthMapDebugImages::displayUpdateKeyFrame() {
    std::scoped_lock lock(_lock);
    Util::displayImage( "Stereo Key Frame", _debugImageHypothesisHandling, false );
    Util::displayImage( "Stereo Reference Frame", _debugImageStereoLines, false );
  }


  void DepthMapDebugImages::plotNewKeyFrame( const Frame::SharedPtr &newKeyFrame ) {
    cv::Mat keyFrameImage(_imageSize.height, _imageSize.width, CV_32F, const_cast<float*>(newKeyFrame->image(0)));
    keyFrameImage.convertTo(_debugImageHypothesisPropagation, CV_8UC1);
    cv::cvtColor(_debugImageHypothesisPropagation, _debugImageHypothesisPropagation, cv::COLOR_GRAY2RGB);
  }

  void DepthMapDebugImages::displayNewKeyFrame() {
    Util::displayImage( "KeyFramePropagation", _debugImageHypothesisPropagation );
  }

  //void DepthMapDebugImages::setHypothesisHandling( int x, int y, cv::Vec3b color ) {
  //  _debugImageHypothesisHandling.at<cv::Vec3b>(y, x) = color;
  //}

  //void DepthMapDebugImages::addStereoLine( const cv::Point &a, const cv::Point &b, const cv::Scalar &color ) {
  //  cv::line(_debugImageStereoLines,a,b,color,1,8,0);
  //}

  // TODO.  Should expire this version
  int DepthMapDebugImages::debugPlotDepthMap( const Frame::SharedPtr &activeKeyFrame,  DepthMapPixelHypothesis *currentDepthMap, int refID, const char *buf1, const char *buf2 )
  {
    if(activeKeyFrame == 0) return 1;

    cv::Mat keyFrameImage(_imageSize.height, _imageSize.width, CV_32F, const_cast<float*>(activeKeyFrame->image(0)));
    cv::Mat keyFrameGray( keyFrameImage.size(), CV_8UC1 );
    keyFrameImage.convertTo(keyFrameGray, CV_8UC1);
    cv::cvtColor(keyFrameGray, _debugImageDepth, cv::COLOR_GRAY2RGB);

    // debug plot & publish sparse version?

    // TODO.  Fix this ...
    //int refID = referenceFrameByID_offset;


    for(int y=0;y<(_imageSize.height);y++)
    for(int x=0;x<(_imageSize.width);x++)
    {
      int idx = x + y*_imageSize.width;

      if(currentDepthMap[idx].blacklisted < MIN_BLACKLIST && Conf().debugDisplay == 2)
      _debugImageDepth.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,255);

      if(!currentDepthMap[idx].isValid) continue;

      cv::Vec3b color = currentDepthMap[idx].getVisualizationColor(refID);
      _debugImageDepth.at<cv::Vec3b>(y,x) = color;
    }

    printMessageOnCVImage(_debugImageDepth, buf1, buf2);

    return 1;
  }


  int DepthMapDebugImages::debugPlotDepthMap( const Frame::SharedPtr &activeKeyFrame, const DepthMapPixelHypothesisVector &currentDepthMap, int refID, const char *buf1, const char *buf2 )
  {
    if(activeKeyFrame == 0) return 1;

    cv::Mat keyFrameImage(_imageSize.height, _imageSize.width, CV_32F, const_cast<float*>(activeKeyFrame->image(0)));
    cv::Mat keyFrameGray( keyFrameImage.size(), CV_8UC1 );
    keyFrameImage.convertTo(keyFrameGray, CV_8UC1);
    cv::cvtColor(keyFrameGray, _debugImageDepth, cv::COLOR_GRAY2RGB);

    // debug plot & publish sparse version?

    // TODO.  Fix this ...
    //int refID = referenceFrameByID_offset;


    for(int y=0;y<(_imageSize.height);y++)
    for(int x=0;x<(_imageSize.width);x++)
    {
      int idx = x + y*_imageSize.width;

      if(currentDepthMap[idx].blacklisted < MIN_BLACKLIST && Conf().debugDisplay == 2)
      _debugImageDepth.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,255);

      if(!currentDepthMap[idx].isValid) continue;

      cv::Vec3b color = currentDepthMap[idx].getVisualizationColor(refID);
      _debugImageDepth.at<cv::Vec3b>(y,x) = color;
    }

    printMessageOnCVImage(_debugImageDepth, buf1, buf2);

    return 1;
  }


}
