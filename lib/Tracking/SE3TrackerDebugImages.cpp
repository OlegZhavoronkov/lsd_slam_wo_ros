#include <fmt/ostream.h>
#include <fmt/printf.h>
#include "util/SophusUtil.h"
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
    //_debugImageWeights              =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _debugImageResiduals            =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _debugImageSecondFrame          =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _debugImageOldImageWarped       =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _debugImageOldImageSource       =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;

    //_backDebugImageWeights          =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _backDebugImageResiduals        =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _backDebugImageSecondFrame      =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _backDebugImageOldImageWarped   =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
    _backDebugImageOldImageSource   =cv::Mat(_lastSize,CV_8UC3,{0,0,0})           ;
}

void SE3TrackerDebugImages::calcResidualAndBuffers_debugStart(cv::Mat*&  pDebugImageOldImageSource,   cv::Mat*&  pDebugImageOldImageWarped,   
                                                        cv::Mat*&  pDebugImageResiduals)
{
    //if(_saveAllTrackingStagesInternal)
    //{
    int other = _saveAllTrackingStagesInternal ? 255 : 0;
    //fillCvMat(&_debugImageResiduals,cv::Vec3b(255,112,0));
    //fillCvMat(&_debugImageWeights,cv::Vec3b(other,other,255));
    //fillCvMat(&_debugImageOldImageSource,cv::Vec3b(other,other,255));
    //fillCvMat(&_debugImageOldImageWarped,cv::Vec3b(other,other,255));   
    
    fillCvMat(&_backDebugImageResiduals,cv::Vec3b(255,112,0));
    //fillCvMat(&_backDebugImageWeights,cv::Vec3b(other,other,255));
    fillCvMat(&_backDebugImageOldImageSource,cv::Vec3b(other,other,255));
    fillCvMat(&_backDebugImageOldImageWarped,cv::Vec3b(other,other,255));
    if(pDebugImageOldImageSource==nullptr && pDebugImageOldImageWarped==nullptr && pDebugImageResiduals==nullptr)
    {
        pDebugImageOldImageSource=&_backDebugImageOldImageSource;
        pDebugImageOldImageWarped=&_backDebugImageOldImageWarped;
        pDebugImageResiduals=&_backDebugImageResiduals;
    }
}


void SE3TrackerDebugImages::calcResidualAndBuffers_debugFinish( int     w                   ,
                                                                int     loop                ,
                                                                int     buf_warped_size     ,
                                                                int     goodCount           ,
                                                                int     badCount            ,
                                                                float   ratio               )
{
    _warpedDebugStringsImage=getTextMat(_warpedDebugStringsImage,
                                        fmt::v7::format("w {0}\nloop {1}\nbuf_warped_size {2}\ngoodCount {3}\nbadCount {4} \nratio {5}"
                                                    ,w              
                                                    ,loop           
                                                    ,buf_warped_size
                                                    ,goodCount      
                                                    ,badCount       
                                                    ,ratio          ),cv::FONT_HERSHEY_COMPLEX,0.5,0.5,cv::Scalar{255,255,255});
    auto warpedSize=cv::Size(   std::max(
                                            _backDebugImageOldImageWarped.cols,
                                            _warpedDebugStringsImage.cols),
                                std::max(   _debugImageOldImageWarped.rows,
                                            _backDebugImageOldImageWarped.rows+_warpedDebugStringsImage.rows)
                            );
    if(warpedSize.height!=_debugImageOldImageWarped.rows || warpedSize.width!=_debugImageOldImageWarped.cols)
    {
        _debugImageOldImageWarped = cv::Mat(warpedSize,CV_8UC3,{0,0,0});
    }
    _backDebugImageOldImageWarped.copyTo(_debugImageOldImageWarped(cv::Rect(0,0,_backDebugImageOldImageSource.cols,_backDebugImageOldImageWarped.rows)));
    _warpedDebugStringsImage.copyTo( _debugImageOldImageWarped(cv::Rect(0,_backDebugImageOldImageWarped.rows,_warpedDebugStringsImage.cols,_warpedDebugStringsImage.rows)));

    _backDebugImageOldImageSource.copyTo(_debugImageOldImageSource);
    

    //Util::displayImage( "SE3Tracker Weights",                                                   _debugImageWeights );
    
    Util::displayImage( "SE3Tracker Intensities of second_frame at transformed positions",      _debugImageOldImageSource );
    Util::displayImage( "SE3Tracker Intensities of second_frame at pointcloud in first_frame",  _debugImageOldImageWarped );
    
       
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

void SE3TrackerDebugImages::Se3TrackerSetSecondFrame(const cv::Size& sz,const float* pSecondFrame,int frameid,int keyFrameId)
{
    if(_debugImageSecondFrame.cols != sz.width || _debugImageSecondFrame.rows!=sz.height)
    {
        _debugImageSecondFrame=cv::Mat(sz,CV_8UC3,{0,0,0});
    }
    cv::Mat(sz.height,sz.width,CV_32FC1,const_cast<void*>(reinterpret_cast<const void*>(pSecondFrame ))).convertTo(_debugImageSecondFrame,CV_8UC3);
    cv::putText(_debugImageSecondFrame,fmt::v7::format("frame {0} key_frame {1}",frameid,keyFrameId).c_str(),cv::Point(10,10),cv::FONT_HERSHEY_COMPLEX,0.5,{255,255,255},1);
    Util::displayImage( "SE3Tracker second_frame", _debugImageSecondFrame );
}

void SE3TrackerDebugImages::Se3TrackingFinishedDisplayResiduals(float residual,const Sophus::SE3f& refToFrame,const Sophus::SE3f& keyToWorld,const std::vector<int>& iterations)
{
    auto frameToWorld=(keyToWorld*refToFrame);
    auto se3formatter=[&](const auto& val)->std::string
                        {
                            std::stringstream str;
                            str << val;
                            return str.str();
                        };
    _residualsFinishResults=getTextMat(_residualsFinishResults,
                                        fmt::v7::format("residual {0}\niterations:\n{1}\nref to frame {2}\nref to world {3}\nframe to world {4}"
                                                    ,residual              
                                                    ,([&]()->auto
                                                    {
                                                        std::string ss;
                                                        size_t idx=0;
                                                        for(const auto& iter:iterations)
                                                        {
                                                            ss=ss.empty()? fmt::v7::format("{0} {1}",idx++,iter):fmt::v7::format("{0}\n{1} {2}",ss,idx++,iter);
                                                        }
                                                        return ss;
                                                    })()           
                                                    ,se3formatter(refToFrame    )  
                                                    ,se3formatter(keyToWorld)
                                                    ,se3formatter(frameToWorld  )     ),
                                                    cv::FONT_HERSHEY_COMPLEX,0.5,0.5,cv::Scalar{255,255,255});
    _backDebugImageResiduals.copyTo(_debugImageResiduals);
    auto resSize=cv::Size(   std::max(
                                            _backDebugImageResiduals.cols,
                                            _residualsFinishResults.cols),
                                std::max(   _debugImageResiduals.rows,
                                            _backDebugImageResiduals.rows+_residualsFinishResults.rows)
                            );
    if(resSize.height!=_debugImageResiduals.rows || resSize.width!=_debugImageResiduals.cols)
    {
        _debugImageResiduals = cv::Mat(resSize,CV_8UC3,{0,0,0});
    }
    _backDebugImageResiduals.copyTo(_debugImageResiduals (cv::Rect(0,0,_backDebugImageResiduals.cols,_backDebugImageResiduals.rows)));
    _residualsFinishResults.copyTo( _debugImageResiduals(cv::Rect(0,_backDebugImageResiduals.rows,_residualsFinishResults.cols,_residualsFinishResults.rows)));
    Util::displayImage( "SE3Tracker Residuals",                                                 _debugImageResiduals );
}

}

