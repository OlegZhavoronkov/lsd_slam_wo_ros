

#include "ImageSet.h"

namespace lsd_slam {

  ImageSet::ImageSet( unsigned int id, const cv::Mat &img,
                      const libvideoio::Camera &cam )
    : _refFrame(0)
  {
    _frames.push_back( std::make_shared<Frame>(id, cam, img.size(), 0.0, img.data ) );
    _se3FromFirst.push_back( SE3() );
    _frameId = id;
  }

  ImageSet::ImageSet(unsigned int id, const cv::Mat &imgL, const cv::Mat &imgR,
                     const libvideoio::Camera &camL,
                     const libvideoio::Camera &camR, const unsigned int ref) {
    _frames.push_back(
        std::make_shared<Frame>(id, camL, imgL.size(), 0.0, imgL.data));
    _frames.push_back(
        std::make_shared<Frame>(id, camR, imgR.size(), 0.0, imgR.data));
    _se3FromFirst.push_back(Sophus::SE3());
    _frameId = id;
    _refFrame = ref;
  }

ImageSet::~ImageSet() {}

void ImageSet::pushbackFrame(const cv::Mat &img,
                             const libvideoio::Camera &cam) {
  _frames.push_back(
      std::make_shared<Frame>(_frameId, cam, img.size(), 0.0, img.data));
}

} // namespace lsd_slam
