#pragma once
#include <opencv2/core/mat.hpp>
#include <libvideoio/types/ImageSize.h>
#include <mutex>
namespace lsd_slam
{

class SE3TrackerDebugImages 
{
public:
	SE3TrackerDebugImages() = delete;
	SE3TrackerDebugImages( const SE3TrackerDebugImages & ) = delete;

	SE3TrackerDebugImages( const libvideoio::ImageSize& imgSize,bool saveAllTrackingStagesInternal=false );
    void calcResidualAndBuffers_debugStart();
    void calcResidualAndBuffers_debugFinish( int     w                   ,
                                                        int     loop                ,
                                                        int     buf_warped_size     ,
                                                        int     goodCount           ,
                                                        int     badCount            ,
                                                        float   ratio               );
private:
    void Initialize();
	// debug images
public:
	cv::Mat _debugImageWeights;
	cv::Mat _debugImageResiduals;
	cv::Mat _debugImageSecondFrame;
	cv::Mat _debugImageOldImageWarped;
	cv::Mat _debugImageOldImageSource;
private:
	cv::Mat _backDebugImageWeights;
	cv::Mat _backDebugImageResiduals;
	cv::Mat _backDebugImageSecondFrame;
	cv::Mat _backDebugImageOldImageWarped;
	cv::Mat _backDebugImageOldImageSource;
public:
    std::mutex _mtx;
    cv::Size _lastSize;
private:
    bool _saveAllTrackingStagesInternal;
};

}