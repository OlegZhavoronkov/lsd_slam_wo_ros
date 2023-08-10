#include "SE3TrackerDebugImages.h"
#include "util/globalFuncs.h"
#include "IOWrapper/ImageDisplay.h"
#include <opencv2/highgui/highgui.hpp>
#include <fmt/format.h>
namespace lsd_slam
{

SE3TrackerDebugImages::SE3TrackerDebugImages( const libvideoio::ImageSize& imgSize,bool saveAllTrackingStagesInternal)
    :   _lastSize(imgSize.cvSize()),
        _saveAllTrackingStagesInternal(saveAllTrackingStagesInternal)
{
    if(_lastSize.width>0 && _lastSize.height>0)
    {
        Initialize();
    }
}

void SE3TrackerDebugImages::Initialize()
{
    _debugImageWeights              =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _debugImageResiduals            =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _debugImageSecondFrame          =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _debugImageOldImageWarped       =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _debugImageOldImageSource       =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;

    _backDebugImageWeights          =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _backDebugImageResiduals        =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _backDebugImageSecondFrame      =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _backDebugImageOldImageWarped   =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _backDebugImageOldImageSource   =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
}

void SE3TrackerDebugImages::calcResidualAndBuffers_debugStart()
{
    if(_saveAllTrackingStagesInternal)
    {
        int other = _saveAllTrackingStagesInternal ? 255 : 0;
        fillCvMat(&_debugImageResiduals,cv::Vec3b(255,112,0));
        fillCvMat(&_debugImageWeights,cv::Vec3b(other,other,255));
        fillCvMat(&_debugImageOldImageSource,cv::Vec3b(other,other,255));
        fillCvMat(&_debugImageOldImageWarped,cv::Vec3b(other,other,255));   

        fillCvMat(&_backDebugImageResiduals,cv::Vec3b(255,112,0));
        fillCvMat(&_backDebugImageWeights,cv::Vec3b(other,other,255));
        fillCvMat(&_backDebugImageOldImageSource,cv::Vec3b(other,other,255));
        fillCvMat(&_backDebugImageOldImageWarped,cv::Vec3b(other,other,255));
    }
}


void SE3TrackerDebugImages::calcResidualAndBuffers_debugFinish( int     w                   ,
                                                                int     loop                ,
                                                                int     buf_warped_size     ,
                                                                int     goodCount           ,
                                                                int     badCount            ,
                                                                float   ratio               )
{

    cv::putText(_debugImageOldImageWarped,fmt::v7::format("w {0}\nloop {1}\nbuf_warped_size {2}\ngoodCount {3}\nbadCount {4} \nratio {5}",w              
,loop           
,buf_warped_size
,goodCount      
,badCount       
,ratio          ).c_str(),cv::Point(10,20),cv::FONT_HERSHEY_COMPLEX,1,{0,255,0});
    Util::displayImage( "SE3Tracker Weights",                                                   _debugImageWeights );
    Util::displayImage( "SE3Tracker second_frame",                                              _debugImageSecondFrame );
    Util::displayImage( "SE3Tracker Intensities of second_frame at transformed positions",      _debugImageOldImageSource );
    Util::displayImage( "SE3Tracker Intensities of second_frame at pointcloud in first_frame",  _debugImageOldImageWarped );
    Util::displayImage( "SE3Tracker Residuals",                                                 _debugImageResiduals );
       
    //if(saveAllTrackingStagesInternal)
    //{
    //    char charbuf[500];  
    //    snprintf(charbuf,500,"save/%sresidual-%d-%d.png",packagePath.c_str(),w,iterationNumber);
    //    cv::imwrite(charbuf,            _debugImageResiduals);  
    //    snprintf(charbuf,500,"save/%swarped-%d-%d.png",packagePath.c_str(),w,iterationNumber);
    //    cv::imwrite(charbuf,            _debugImageOldImageWarped); 
    //    snprintf(charbuf,500,"save/%sweights-%d-%d.png",packagePath.c_str(),w,iterationNumber);
    //    cv::imwrite(charbuf,            _debugImageWeights);    
    //    printf("saved three images for lvl %d, iteration %d\n",w,iterationNumber);
    //}
}



}

