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

#include <shared_mutex>

#include "TrackingThread.h"

#include "SlamSystem.h"

#include "DataStructures/KeyFrame.h"
#include "Tracking/SE3Tracker.h"
// #include "Tracking/Sim3Tracker.h"
// #include "DepthEstimation/DepthMap.h"
#include "Tracking/TrackingReference.h"
// #include "util/globalFuncs.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
//#include "ros/ros.h"
// #include "GlobalMapping/g2oTypeSim3Sophus.h"
// #include "IOWrapper/ImageDisplay.h"
// #include "IOWrapper/Output3DWrapper.h"
// #include <g2o/core/robust_kernel_impl.h>
// #include "DataStructures/FrameMemory.h"
// #include "deque

#include "SlamSystem/MappingThread.h"

// for mkdir
#include <sys/stat.h>
#include <sys/types.h>

#include <g3log/g3log.hpp>

#ifdef ANDROID
#include <android/log.h>
#endif

#include "opencv2/opencv.hpp"
#include "util/SophusUtil.h"
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/printf.h>
using namespace lsd_slam;

using active_object::Active;

void TrackingThread::OnSignalConnectedHandler(const util::SE3TrackerSignals* pSignals,const boost::signals2::signal_base* pToSignal,bool connected)
{
    util::SE3TrackerSignals* pThisRoot=static_cast<util::SE3TrackerSignals*>(this);
    if(pThisRoot==pSignals && connected)
    {
        CHECK_CONNECT_THIS_CHAIN_TO_SAME_NAME(_OnCalcResidualErrorCalculatedSignal,pToSignal,*_tracker)
        CHECK_CONNECT_THIS_CHAIN_TO_SAME_NAME(_OnCalcResidualFinishedSignal,pToSignal,*_tracker)
        CHECK_CONNECT_THIS_CHAIN_TO_SAME_NAME(_OnCalcResidualStartedSignal,pToSignal,*_tracker)
        CHECK_CONNECT_THIS_CHAIN_TO_SAME_NAME(_OnCalcResidualAndBuffersDebugStart,pToSignal,*_tracker)
        CHECK_CONNECT_THIS_CHAIN_TO_SAME_NAME(_OnCalcResidualAndBuffersDebugFinish,pToSignal,*_tracker)

    }
    
}

TrackingThread::TrackingThread( SlamSystem &system, bool threaded )
: _system( system ),
	_perf(),
	_tracker( new SE3Tracker( Conf().slamImageSize ) ),
	//_trackingReference( new TrackingReference() ),
	_trackingIsGood( true ),
	_newKeyFramePending( false ),
	_latestGoodPoseCamToWorld(),
	_thread( threaded ? Active::createActive() : NULL )
{
    _onSignalConnected.connect([this](auto...args){this->OnSignalConnectedHandler(args...);});
	for (int level = 4; level < PYRAMID_LEVELS; ++level)
		_tracker->settings.maxItsPerLvl[level] = 0;

	lastTrackingClosenessScore = 0;
    //Connect(*const_cast<TrackingThread*>(this),&TrackingThread::_OnCalcResidualErrorCalculatedSignal,[&](auto.../*args*/)->auto{},&_onSignalConnected);
}


TrackingThread::~TrackingThread()
{;}

void TrackingThread::trackSetImpl( const std::shared_ptr<ImageSet> &set )
{
    try
    {
        trackSetImplInternal(set);
    }
    catch(const std::exception& ex)
    {
        LOGF(WARNING,"exception for frame %u:\n%s",set->id(),ex.what());
    }
    catch(...)
    {
        LOGF(WARNING,"unknown exception for frame %u",set->id());
    }
}

void TrackingThread::trackSetImplInternal( const std::shared_ptr<ImageSet> &set )
{
	if(!_trackingIsGood) {
	        // Prod mapping to check the relocalizer

					//!!TODO.  Fix this
	        // _system._mapThread->relocalizer.updateCurrentFrame(set->refFrame());
	        // _system._mapThread->pushDoIteration();

	        return;
	}

	// DO TRACKING & Show tracking result.
	LOG_IF(DEBUG, Conf().print.threadingInfo) << "TRACKING frame " << set->refFrame()->id() << " onto ref. " << _currentKeyFrame->id();
    
//    LOGF(WARNING,"tracking frame id %d _latestGoodPoseCamToWorld %s _currentKeyFrame->pose()->getCamToWorld() %s",
//        set->refFrame()->id(),
//        fmt::v7::sprintf("%s",_latestGoodPoseCamToWorld).c_str(),
//        fmt::v7::sprintf("%s",_currentKeyFrame->pose()->getCamToWorld()).c_str());
	SE3 frameToReference_initialEstimate =se3FromSim3(  _currentKeyFrame->pose()->getCamToWorld().inverse() * _latestGoodPoseCamToWorld);
    LOG(WARNING)<< "tracking frame id " << set->refFrame()->id() 
                << "\n _latestGoodPoseCamToWorld " << _latestGoodPoseCamToWorld 
                << "\n_currentKeyFrame->pose()->getCamToWorld() "<< _currentKeyFrame->pose()->getCamToWorld()
                << "\n_currentKeyFrame->pose()->getCamToWorld().inverse() " << _currentKeyFrame->pose()->getCamToWorld().inverse()
                << "\nframeToReference_initialEstimate " << frameToReference_initialEstimate;
	Timer timer;

	LOGF(DEBUG, "Start tracking..._currentKeyFrame %d frame to track %d frameToReference_initialEstimate trans [%f %f %f] %f",
            currentKeyFrame()->id(),
            set->refFrame()->id(),
            frameToReference_initialEstimate.translation()[0],
            frameToReference_initialEstimate.translation()[1],
            frameToReference_initialEstimate.translation()[2],
            frameToReference_initialEstimate.translation().norm()
            );
	SE3 newRefToFrame_poseUpdate = _tracker->trackFrame( _currentKeyFrame,
	                                                    set->refFrame(),
	                                                    frameToReference_initialEstimate);
    LOG(WARNING)<< " after tracking frame id " << set->refFrame()->id() 
                << "\nnewRefToFrame_poseUpdate " << newRefToFrame_poseUpdate;
                
    LOGF(DEBUG, "Done tracking,took %.1f ms to track frame %d  to  %d newRefToFrame_poseUpdate trans [%f %f %f] norm %f",
            timer.stop()*1000,
            set->refFrame()->id(),
            currentKeyFrame()->id(),
            newRefToFrame_poseUpdate.translation()[0],
            newRefToFrame_poseUpdate.translation()[1],
            newRefToFrame_poseUpdate.translation()[2],
            newRefToFrame_poseUpdate.translation().norm()
            );
	
	_perf.track.update( timer );


	tracking_lastResidual = _tracker->lastResidual;
	tracking_lastUsage = _tracker->pointUsage;

	if(manualTrackingLossIndicated || _tracker->_diverged ||
	        (_system.keyFrameGraph()->keyframesAll.size() > INITIALIZATION_PHASE_COUNT && !_tracker->_trackingWasGood))
	{
	        LOGF(WARNING, "TRACKING LOST for frame %d (%1.2f%% good Points, which is %1.2f%% of available points; %s tracking; tracker has %s)!\n",
	                        set->refFrame()->id(),
	                        100*_tracker->_pctGoodPerTotal,
	                        100*_tracker->_pctGoodPerGoodBad,
	                        _tracker->_trackingWasGood ? "GOOD" : "BAD",
	                        _tracker->_diverged ? "DIVERGED" : "NOT DIVERGED");

	        //_trackingReference->invalidate();
	        setTrackingIsBad();

					//!!TODO.  What does mapping thread do while tracking is bad?
	        //_system.mapThread->pushDoIteration();
	        manualTrackingLossIndicated = false;
	        return;
	}
	_latestGoodPoseCamToWorld = set->refFrame()->pose->getCamToWorld();

	LOG_IF( DEBUG,  Conf().print.threadingInfo ) << "Publishing tracked frame";
	_system.publishTrackedFrame(set->refFrame());
    LOGF(WARNING,"tracking frame %d  with keyFrame %d quat norm %f",
            set->id(),
            (_currentKeyFrame!=nullptr ? _currentKeyFrame->id():-1),
            sqrt(set->refFrame()->getCamToWorld().quaternion().squaredNorm()));
	_system.publishPose(set->refFrame()->getCamToWorld().cast<float>());

	// Keyframe selection
	LOG(INFO) << "Tracked " << set->id() << " against keyframe " << _currentKeyFrame->id();
	LOG_IF( INFO, Conf().print.threadingInfo ) << _currentKeyFrame->numMappedOnThisTotal << " frames mapped on to keyframe " << _currentKeyFrame->id() << ", considering " << set->refFrame()->id() << " as new keyframe.";

	// Push to mapping before checking if its the new keyframe
	_system.mapThread()->doMapSet( _currentKeyFrame, set );

	if( !_newKeyFramePending && _currentKeyFrame->numMappedOnThisTotal > MIN_NUM_MAPPED)
	{
	  Sophus::Vector3d dist = newRefToFrame_poseUpdate.translation() * _currentKeyFrame->frame()->meanIdepth;
#if 0 //только для отладки алгоритма,потом надо вернуть назад так как непоянтна эфристикка выбора ключевого кадра
	  float minVal = 0;
#else//
        float minVal = fmin(0.2f + _system.keyFrameGraph()->size() * 0.8f / INITIALIZATION_PHASE_COUNT,1.0f);
#endif

	  if(_system.keyFrameGraph()->size() < INITIALIZATION_PHASE_COUNT)
      {
        minVal *= 0.7;
      }	

	  lastTrackingClosenessScore = _system.trackableKeyFrameSearch()->getRefFrameScore(dist.dot(dist), _tracker->pointUsage);

	  if (lastTrackingClosenessScore > minVal || _currentKeyFrame->numMappedOnThisTotal > (MIN_NUM_MAPPED+1))
	  {
	    LOG(INFO) << "Telling mapping thread to make " << set->refFrame()->id() << " the new keyframe.";

			_newKeyFramePending = true;
	    _system.mapThread()->doCreateNewKeyFrame( _currentKeyFrame, set->refFrame() );

	    LOGF_IF( INFO, Conf().print.keyframeSelectionInfo,
	                                    "SELECT KEYFRAME %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",set->refFrame()->id(),set->refFrame()->trackingParent()->id(), dist.dot(dist), _tracker->pointUsage, _system.trackableKeyFrameSearch()->getRefFrameScore(dist.dot(dist), _tracker->pointUsage));
	  }
	  else
	  {
	    LOGF_IF( INFO, Conf().print.keyframeSelectionInfo,
	                                    "SKIPPD KEYFRAME %d on %d! dist %.3f + usage %.3f = %.3f > 1\n",set->refFrame()->id(),set->refFrame()->trackingParent()->id(), dist.dot(dist), _tracker->pointUsage, _system.trackableKeyFrameSearch()->getRefFrameScore(dist.dot(dist), _tracker->pointUsage));
	  }
	}

	LOG_IF( DEBUG, Conf().print.threadingInfo ) << "Exiting trackFrame";

}

void TrackingThread::useNewKeyFrameImpl( const std::shared_ptr<KeyFrame> &kf )
{
	LOG(DEBUG) << "Using " << kf->id() << " as new keyframe";
	_newKeyFramePending = false;
	_currentKeyFrame = kf;
}

// n.b. this function will be called from the mapping thread.  Ensure
// locking is in place.

//TODO I don't think this is ever entered?
//Need to add pushUnmappedTrackedFrame for image set
void TrackingThread::takeRelocalizeResult( const RelocalizerResult &result  )
{
        LOG(WARNING) << "Entering takeRelocalizeResult";
	// Frame* keyframe;
	// int succFrameID;
	// SE3 succFrameToKF_init;
	// std::shared_ptr<Frame> succFrame;
	//
	// relocalizer.stop();
	// relocalizer.getResult(keyframe, succFrame, succFrameID, succFrameToKF_init);
	// assert(keyframe != 0);

	//KeyFrame::SharedPtr keyframe( _currentKeyFrame );
	// _trackingReference->importFrame( keyframe );
	// _trackingReferenceFrameSharedPT = keyframe;

	_tracker->trackFrame(
			_currentKeyFrame,
			result.successfulFrame,
			result.successfulFrameToKeyframe );

	if(!_tracker->_trackingWasGood || _tracker->lastGoodCount() / (_tracker->lastGoodCount()) < 1-0.75f*(1-MIN_GOODPERGOODBAD_PIXEL))
	{
		LOG_IF(DEBUG, Conf().print.relocalizationInfo) << "RELOCALIZATION FAILED BADLY! discarding result.";
		//_trackingReference->invalidate();
	}
	else
	{
		//_system.keyFrameGraph()->addFrame(result.successfulFrame );

                //TODO commenting this out in the assumption I don't need it... need to revist
                //_system.mapThread->pushUnmappedTrackedFrame( result.successfulFrame );

                 //{
                 //       std::lock_guard<std::mutex> lock( currentKeyFrameMutex );
			// createNewKeyFrame = false;
			setTrackingIsGood();
                //}
	}

}
