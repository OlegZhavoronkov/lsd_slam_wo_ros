/**
 * This file is part of LSD-SLAM.
 *
 * Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical
 * University of Munich) For more information see
 * <http://vision.in.tum.de/lsdslam>
 *
 * LSD-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LSD-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#include <memory>

#include "SlamSystem.h"

#include <shared_mutex>

#include <g3log/g3log.hpp>

#ifdef ANDROID
#include <android/log.h>
#endif

#include <opencv2/opencv.hpp>

#include "DataStructures/Frame.h"
#include "DataStructures/FrameMemory.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
#include "util/globalFuncs.h"

#include "SlamSystem/ConstraintSearchThread.h"
#include "SlamSystem/MappingThread.h"
#include "SlamSystem/OptimizationThread.h"

using namespace lsd_slam;

SlamSystem::SlamSystem( )
  : _perf(),
  	_outputWrappers( ),
  	_initialized( false ),
	_finalized(),
  //	_keyFrames(),
  	_keyFrameGraph( new KeyFrameGraph ),
  	_trackableKeyFrameSearch( new TrackableKeyFrameSearch( _keyFrameGraph ) ),
    _pDepthMapDebugImages(plotStereoImages ? new DepthMapDebugImages(Conf().slamImageSize):nullptr)
{

	// Because some of these rely on Conf(), need to explicitly call after
 	// static initialization.  Is this true?
	const bool threaded = Conf().runRealTime;
	_optThread.reset( new OptimizationThread( *this, threaded ) );
	_mapThread.reset( new MappingThread( *this, threaded ) );
	_constraintThread.reset( new ConstraintSearchThread( *this, threaded ) );
	_trackingThread.reset( new TrackingThread( *this, threaded ) );
    if(_pDepthMapDebugImages)
    {
        _mapThread->_depthMapDisplayUpdateKeyFrameSignal.connect([this](auto...args){this->_pDepthMapDebugImages->displayUpdateKeyFrame();});
        _mapThread->_depthMapUpdateKeyFrameSignal.connect(
                [this](auto...args){
                    auto& currKeyFrame=this->currentKeyFrame();
                    this->_pDepthMapDebugImages->setHypotesisAndLineImages(currKeyFrame!=nullptr ? currKeyFrame->frame() :
                                                                                                    Frame::SharedPtr{},
                                                                            args...);});
    }
    if(plotTrackingIterationInfo)
    {
        _pSE3TrackerDebugImages.reset(new SE3TrackerDebugImages(Conf().slamImageSize));
        //TrackingThread::Connect(*_trackingThread,&TrackingThread::_OnCalcResidualErrorCalculatedSignal,[](auto...){},&_trackingThread->_onSignalConnected);
        //TrackingThread::Connect(*_trackingThread,&TrackingThread::_OnCalcResidualStartedSignal,std::bind_front(&SlamSystem::OnSe3TrackerStarted,this),&_trackingThread->_onSignalConnected);
        //TrackingThread::Connect(*_trackingThread,&TrackingThread::_OnCalcResidualFinishedSignal,std::bind_front(&SlamSystem::OnSe3TrackerFinished,this),&_trackingThread->_onSignalConnected);
        TrackingThread::Connect(*_trackingThread,&TrackingThread::_OnCalcResidualAndBuffersDebugStart,std::bind_front(&SlamSystem::OnSe3TrackerCalcResidualAndBuffersDebugStart,this),&_trackingThread->_onSignalConnected);
        TrackingThread::Connect(*_trackingThread,&TrackingThread::_OnCalcResidualAndBuffersDebugFinish ,std::bind_front(&SlamSystem::OnSe3TrackerCalcResidualAndBuffersDebugFinish,this),&_trackingThread->_onSignalConnected);
        TrackingThread::Connect(*_trackingThread,&TrackingThread::_OnSetSecondFrame ,std::bind_front(&SlamSystem::OnSe3TrackerSetSecondFrame,this),&_trackingThread->_onSignalConnected);
        TrackingThread::Connect(*_trackingThread,&TrackingThread::_OnTrackingFinishedDisplayResiduals ,std::bind_front(&SlamSystem::OnSe3TrackingFinishedDisplayResiduals,this),&_trackingThread->_onSignalConnected);
    }
	timeLastUpdate.start();

}

SlamSystem::~SlamSystem() {
  // make sure no-one is waiting for something.
  LOG(INFO) << "... waiting for all threads to exit";

	_mapThread.reset();
	_constraintThread.reset();
	_optThread.reset();
	_trackingThread.reset();
	LOG(INFO) << "DONE waiting for all threads to exit";

	FrameMemory::getInstance().releaseBuffers();
}


SlamSystem *SlamSystem::fullReset( void )
{
	SlamSystem *newSystem = new SlamSystem( );
	for( auto &wrapper : _outputWrappers ) { newSystem->addOutputWrapper( wrapper ); }
	return newSystem;
}

void SlamSystem::finalize()
{
	LOG(INFO) << "Finalizing Graph... adding final constraints!!";

	// Run this in the foreground
	_constraintThread->doFullReConstraintSearch();
	_constraintThread->fullReConstraintSearchComplete.wait();

	LOG(INFO) << "Finalizing Graph... optimizing!!";

	// This happens in the foreground
	// This will kick off a final map publication with the newly optimized offsets (also in foreground)
	_optThread->doFinalOptimization();

	_optThread->finalOptimizationComplete.wait();
	_mapThread->optimizationUpdateMerged.wait();

	LOG(INFO) << "Done Finalizing Graph.!!";
	_finalized.notify();

  LOG(INFO) << "Done Finalizing Graph.!!";
  _finalized.notify();
}

//std::shared_ptr<KeyFrame> &SlamSystem::currentKeyFrame()


// Thin wrapper which turns a bare cv::Mat image into an ImageSe
void SlamSystem::nextImage( unsigned int id, const cv::Mat &img, const libvideoio::Camera &cam )
{
	nextImageSet( std::make_shared<ImageSet>(id, img, cam) );
}


void SlamSystem::nextImageSet( const std::shared_ptr<ImageSet> &set )
{
	 if( !_initialized ) {
		_mapThread->createFirstKeyFrame( set->refFrame() );
		_initialized = true;
	 	return;
	}

	_trackingThread->doTrackSet( set );

	logPerformanceData();
}

//=== Keyframe maintenance functions ====

// void SlamSystem::addKeyFrame( const KeyFrame::SharedPtr &keyframe )
// {
// 	_keyFrames.push_back( keyframe );
//
//
// 	keyFrameGraph()->idToKeyFrame.insert(std::make_pair(keyframe->id(), keyframe));
// }

// void SlamSystem::changeKeyframe( const Frame::SharedPtr &candidate, bool noCreate, bool force, float maxScore)
// {
// 	Frame::SharedPtr newReferenceKF(nullptr);
//
// 	if( Conf().doKFReActivation && Conf().SLAMEnabled )
// 	{
// 		Timer timer;
// 		newReferenceKF = trackableKeyFrameSearch()->findRePositionCandidate( candidate, maxScore );
// 		_perf.findReferences.update( timer );
// 	}
//
// 	if(newReferenceKF != 0) {
// 		LOG(INFO) << "Reloading existing key frame " << newReferenceKF->id();
// 		loadNewCurrentKeyframe(newReferenceKF);
// 	} else {
// 		if(force)
// 		{
// 			if(noCreate)
// 			{
// 				LOG(INFO) << "mapping is disabled & moved outside of known map. Starting Relocalizer!";
// 				trackingThread->setTrackingIsBad();
// 				//nextRelocIdx = -1; /// What does this do?
// 			}
// 			else
// 			{
// 				createNewCurrentKeyframe( candidate );
// 			}
// 		}
// 	}
// 	// createNewKeyFrame = false;
// }
//
// void SlamSystem::loadNewCurrentKeyframe( const Frame::SharedPtr &keyframeToLoad)
// {
// 	depthMap()->activateExistingKF(keyframeToLoad);
//
// 	LOG_IF(DEBUG, Conf().print.regularizeStatistics ) << "re-activate frame " << keyframeToLoad->id() << "!";
//
// 	// Not entirely sure why they're doing this lookup...
// 	//_currentKeyFrame = keyFrameGraph()->idToKeyFrame.find(keyframeToLoad->id())->second;
// 	//currentKeyFrame()->depthHasBeenUpdatedFlag = false;
// }
//
//
// void SlamSystem::createNewCurrentKeyframe( const Frame::SharedPtr &newKeyframeCandidate)
// {
// 	LOG_IF(INFO, Conf().print.threadingInfo) << "CREATE NEW KF " << newKeyframeCandidate->id() << ", replacing " << currentKeyFrame()->id();
//
// 	if( Conf().SLAMEnabled)
// 	{
// 		std::shared_lock< std::shared_mutex > lock( keyFrameGraph()->idToKeyFrameMutex );
// 		keyFrameGraph()->idToKeyFrame.insert(std::make_pair(newKeyframeCandidate->id(), newKeyframeCandidate));
// 	}
//
// 	// propagate & make new.
// 	depthMap()->createKeyFrame(newKeyframeCandidate);
// }

//===== Debugging output functions =====


void SlamSystem::logPerformanceData()
{
	float sPassed = timeLastUpdate.reset();
	if(sPassed > 1.0f)
	{

		// LOGF(DEBUG, "Mapping: %3.1fms (%.1fHz); Track: %3.1fms (%.1fHz); Create: %3.1fms (%.1fHz); FindRef: %3.1fms (%.1fHz); PermaTrk: %3.1fms (%.1fHz); Opt: %3.1fms (%.1fHz); FindConst: %3.1fms (%.1fHz);\n",
		// 			depthMap()->perf().update.ms(), depthMap()->perf().update.rate(),
		// 			_trackingThread->perf().track.ms(),  _trackingThread->perf().track.rate(),
		// 			depthMap()->perf().create.ms()+depthMap()->perf().finalize.ms(), depthMap()->perf().create.rate(),
		// 			_perf.findReferences.ms(), _perf.findReferences.rate(),
		// 			0.0, 0.0,
		// 			//trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.ms() : 0, trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.rate() : 0,
		// 			_optThread->perf.ms(), _optThread->perf.rate(),
		// 			_constraintThread->perf().findConstraint.ms(), _constraintThread->perf().findConstraint.rate() );
		//
		// depthMap()->logPerformanceData();

	}

}

void SlamSystem::updateDisplayDepthMap()
{
	if( !Conf().displayDepthMap ) return;

	// const double scale = (bool)currentKeyFrame() ? currentKeyFrame()->frame()->getCamToWorld().scale() : 1.0;

	// debug plot depthmap
	char buf1[200] = "";
	char buf2[200] = "";

	if( Conf().onSceenInfoDisplay ){
		// snprintf(buf1,200,"Map: Upd %3.0fms (%2.0fHz); Trk %3.0fms (%2.0fHz); %d / %d",
		// 		depthMap()->perf().update.ms(), depthMap()->perf().update.rate(),
		// 		_trackingThread->perf().track.ms(), _trackingThread->perf().track.rate(),
		// 		currentKeyFrame()->numFramesTrackedOnThis, currentKeyFrame()->numMappedOnThis ); //, (int)unmappedTrackedFrames().size());

	// snprintf(buf2,200,"dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/; usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
	// 		100*currentKeyFrame->numPoints/(float)(Conf().slamImage.area()),
	// 		100*tracking_lastGoodPerBad,
	// 		scale,
	// 		tracking_lastResidual,
	// 		100*tracking_lastUsage,
	// 		(int)keyFrameGraph()->allFramePoses.size(),
	// 		keyFrameGraph()->totalVertices,
	// 		(int)keyFrameGraph()->edgesAll.size(),
	// 		1e-6 * (float)keyFrameGraph()->totalPoints);

	}

	currentKeyFrame()->depthMap()->plotDepthMap( buf1, buf2 );

	//CHECK( currentKeyFrame()->depthMap()->debugImages().depthImage().data != NULL );
	//publishDepthImage( currentKeyFrame()->depthMap()->debugImages().depthImage().data );
}


//=== OutputWrapper functions ==

#define OUTPUT_FOR_EACH( func ) \
	for( auto &wrapper  : _outputWrappers ) { wrapper->func; }

void SlamSystem::publishPose(const Sophus::Sim3f &pose ) {
	 OUTPUT_FOR_EACH( publishPose( pose ) )
 }

void SlamSystem::publishTrackedFrame( const Frame::SharedPtr &frame ) {
	OUTPUT_FOR_EACH( publishTrackedFrame( frame ) )
}

void SlamSystem::publishKeyframeGraph( void ) {
	OUTPUT_FOR_EACH( publishKeyframeGraph( keyFrameGraph() ) )
}

void SlamSystem::publishDepthImage( unsigned char* data  ) {
	OUTPUT_FOR_EACH( updateDepthImage( data ) )
}

void SlamSystem::publishKeyframe( const Frame::SharedPtr &frame ) {
	OUTPUT_FOR_EACH( publishKeyframe( frame ) )
}

void SlamSystem::publishCurrentKeyframe( )
{
	if( currentKeyFrame() ) {
		OUTPUT_FOR_EACH( publishKeyframe( currentKeyFrame()->frame() ) )
		OUTPUT_FOR_EACH( publishPointCloud( currentKeyFrame()->frame() ) )
	} else {
		LOG(DEBUG) << "No currentKeyframe, unable to publish";
	}
}


//void SlamSystem::OnSe3TrackerStarted(cv::Mat*& pMatOutput,const cv::Size& sz,std::recursive_mutex& mtx)
//{
//    LOGF(WARNING,"cv::Mat* pMatOutput %p,const cv::Size& [%d %d],std::recursive_mutex& mtx %p",(void*)pMatOutput,sz.width,sz.height,(void*)&mtx);
//    std::scoped_lock lock(mtx);
//    if(pMatOutput==nullptr)
//    {
//        pMatOutput=new cv::Mat(sz,CV_8UC3,{0,0,0});
//    }
//}

//void SlamSystem::OnSe3TrackerFinished(cv::Mat*& pMatOutput,std::recursive_mutex& mtx)
//{
//    LOGF(WARNING,"cv::Mat* pMatOutput %p,std::recursive_mutex& mtx %p",(void*)pMatOutput,(void*)&mtx);
//    std::scoped_lock lock(mtx);
//    if(pMatOutput!=nullptr)
//    {
//        delete pMatOutput;
//        pMatOutput=nullptr;
//    }
//}

void SlamSystem::OnSe3TrackerCalcResidualAndBuffersDebugStart(  const cv::Size& sz,   std::mutex& mtx,
                                                        cv::Mat*&  pDebugImageOldImageSource,   cv::Mat*&  pDebugImageOldImageWarped,   
                                                        cv::Mat*&  pDebugImageResiduals       )
{
    auto pSE3TrackerDebugImages=_pSE3TrackerDebugImages;
    if(!pSE3TrackerDebugImages)
    {
        return;
    }
    std::scoped_lock lock(pSE3TrackerDebugImages->_mtx);
    //if(pDebugImageOldImageSource!=nullptr|| pDebugImageOldImageWarped!=nullptr || pDebugImageResiduals!=nullptr)
    //{
    //    
    //}
    if(sz.width!= pSE3TrackerDebugImages->_lastSize.width && sz.height!= pSE3TrackerDebugImages->_lastSize.height)
    {
        pSE3TrackerDebugImages.reset(new SE3TrackerDebugImages(sz));
        _pSE3TrackerDebugImages=pSE3TrackerDebugImages;
    }
    pSE3TrackerDebugImages->calcResidualAndBuffers_debugStart(pDebugImageOldImageSource,pDebugImageOldImageWarped,pDebugImageResiduals);
    //return;
    //pDebugImageOldImageSource=  & _pSE3TrackerDebugImages->_debugImageOldImageSource;
    //pDebugImageOldImageWarped=  & _pSE3TrackerDebugImages->_debugImageOldImageWarped;
    //pDebugImageResiduals=       & _pSE3TrackerDebugImages->_debugImageResiduals;
}

void SlamSystem::OnSe3TrackerCalcResidualAndBuffersDebugFinish( int     w                   ,
                                                        int     loop                ,
                                                        int     buf_warped_size     ,
                                                        int     goodCount           ,
                                                        int     badCount            ,
                                                        float   ratio               )
{
    auto pSE3TrackerDebugImages=_pSE3TrackerDebugImages;
    if(!pSE3TrackerDebugImages)
    {
        return;
    }
    pSE3TrackerDebugImages->calcResidualAndBuffers_debugFinish(w,loop,buf_warped_size,goodCount,badCount,ratio);
}

void SlamSystem::OnSe3TrackerSetSecondFrame(const cv::Size& sz,const float* secondFrame,int id,int key_id)
{
    auto pSE3TrackerDebugImages=_pSE3TrackerDebugImages;
    if(!pSE3TrackerDebugImages)
    {
        return;
    }
    pSE3TrackerDebugImages->Se3TrackerSetSecondFrame(sz,secondFrame,id,key_id);
}

void SlamSystem::OnSe3TrackingFinishedDisplayResiduals(float residual,Sophus::SE3f refToFrame,Sophus::SE3f keyToWorld,const std::vector<int>& iterations)
{
    auto pSE3TrackerDebugImages=_pSE3TrackerDebugImages;
    if(!pSE3TrackerDebugImages)
    {
        return;
    }
    pSE3TrackerDebugImages->Se3TrackingFinishedDisplayResiduals(residual,refToFrame,keyToWorld,iterations);
}