/**
* This file is derived from Jakob Engel's original LSD-SLAM code.
* His original copyright notice follows:
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
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

#pragma once

#include "active_object/active.h"

#include "DataStructures/KeyFrame.h"
#include "DataStructures/ImageSet.h"
#include "Tracking/Relocalizer.h"
#include "util/MovingAverage.h"
#include "util/SE3TrackerSignals.h"
namespace lsd_slam
{

class SlamSystem;
class SE3Tracker;


typedef Eigen::Matrix<float, 7, 7> Matrix7x7;

class TrackingThread :public util::SE3TrackerSignals
{
friend class IntegrationTest;
public:

	// Delete these
	TrackingThread( const TrackingThread&) = delete;
	TrackingThread& operator=(const TrackingThread&) = delete;

	TrackingThread( SlamSystem &system, bool threaded );

	~TrackingThread();

	//== Calls into the thread ==
  void doTrackSet( const std::shared_ptr<ImageSet> &set ) {
		if( _thread ) {
			_thread->send( std::bind( &TrackingThread::trackSetImpl, this, set ));
		} else {
			trackSetImpl( set );
		}
	}

	void doUseNewKeyFrame( const std::shared_ptr<KeyFrame> &kf ) {
		if( _thread ) {
			_thread->send( std::bind( &TrackingThread::useNewKeyFrameImpl, this, kf ));
		} else {
			useNewKeyFrameImpl( kf );
		}
	}


	KeyFrame::SharedPtr &currentKeyFrame(void) { return _currentKeyFrame; }

	int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent=true, bool useFABMAP=true, float closeCandidatesTH=1.0);

	//void changeKeyframe(std::shared_ptr<Frame> candidate, bool noCreate, bool force, float maxScore);

	void takeRelocalizeResult( const RelocalizerResult &result );

 	float lastTrackingClosenessScore;

	bool trackingIsGood( void ) const { return _trackingIsGood; }
	bool setTrackingIsBad( void )  { return _trackingIsGood = false; }
	bool setTrackingIsGood( void ) { return _trackingIsGood = true; }

	struct PerformanceData {
		MsRateAverage track;
	};

	PerformanceData perf() const { return _perf; }

private:

	SlamSystem &_system;
	PerformanceData _perf;

	std::unique_ptr<SE3Tracker> _tracker;

	// Thread Callbacks
	void trackSetImpl( const std::shared_ptr<ImageSet> &set );
    void trackSetImplInternal( const std::shared_ptr<ImageSet> &set );
	void useNewKeyFrameImpl( const std::shared_ptr<KeyFrame> &kf );

	// ============= EXCLUSIVELY TRACKING THREAD (+ init) ===============

	// std::shared_ptr<TrackingReference> _trackingReference; // tracking reference for current keyframe. only used by tracking.
	// Frame::SharedPtr _trackingReferenceFrameSharedPT;	// only used in odometry-mode, to keep a keyframe alive until it is deleted. ONLY accessed whithin currentKeyFrameMutex lock.

	bool _trackingIsGood;
	bool _newKeyFramePending;

	KeyFrame::SharedPtr _currentKeyFrame;

	Sim3 _latestGoodPoseCamToWorld;


	//
	//
	// // ============= EXCLUSIVELY MAPPING THREAD (+ init) =============
	//
	//
	//
	// // ============= EXCLUSIVELY FIND-CONSTRAINT THREAD (+ init) =============
	//
	//
	//
	//
	// // ============= SHARED ENTITIES =============
	float tracking_lastResidual;
	float tracking_lastUsage;
	float tracking_lastGoodPerBad;
	float tracking_lastGoodPerTotal;

	std::unique_ptr<active_object::Active> _thread;


	//
	// int lastNumConstraintsAddedOnFullRetrack;
	// bool doFinalOptimization;
	//
	// // for sequential operation. Set in Mapping, read in Tracking.
	// // std::condition_variable  newFrameMappedSignal;
	// // std::mutex newFrameMappedMutex;
	//
	//
	//

	//
	//
	//
	//
	//
	// // Tracking: if (!create) set candidate, set create.
	// // Mapping: if (create) use candidate, reset create.
	// // => no locking required.
	// std::shared_ptr<Frame> latestTrackedFrame;
	// bool createNewKeyFrame;
	//
	//
	//
	// // PUSHED in tracking, READ & CLEARED in mapping
	// // std::deque< std::shared_ptr<Frame> > unmappedTrackedFrames;
	// // ThreadSynchronizer unmappedTrackedFramesSynchro;
	// // std::mutex unmappedTrackedFramesMutex;
	// // std::condition_variable  unmappedTrackedFramesSignal;
	//
	//
	// // PUSHED by Mapping, READ & CLEARED by constraintFinder
	// ThreadMutexObject< std::deque< Frame* > > newKeyFrames;
	// // std::deque< Frame* > newKeyFrames;
	// // std::mutex newKeyFrameMutex;
	// // std::condition_variable newKeyFrameCreatedSignal;
	//
	//
	//
	//
	// // threads
	// // std::thread thread_mapping;
	// // std::thread thread_constraint_search;
	// //std::thread thread_optimization;
	//
	// // bool keepRunning; // used only on destruction to signal threads to finish.
	//
	//
	//
	// // optimization thread
	// // bool newConstraintAdded;
	// // std::mutex newConstraintMutex;
	// // std::condition_variable newConstraintCreatedSignal;
	//
	//
	//
	//
	//
	//
	//
	//
	// bool depthMapScreenshotFlag;
	// std::string depthMapScreenshotFilename;
	//

	//

	//
	//
	// void changeKeyframe(bool noCreate, bool force, float maxScore);
	// void createNewCurrentKeyframe(std::shared_ptr<Frame> newKeyframeCandidate);
	// void loadNewCurrentKeyframe(Frame* keyframeToLoad);
	//

	//
	// void constraintSearchThreadLoop();

	// /** Calculates a scale independent error norm for reciprocal tracking results a and b with associated information matrices. */


	//void optimizationThreadLoop();

public:
    //OnCalcResidualStartedSignal         _OnCalcResidualStartedSignal        ;
    //OnCalcResidualFinishedSignal        _OnCalcResidualFinishedSignal       ;
    //OnCalcResidualErrorCalculatedSignal _OnCalcResidualErrorCalculatedSignal;

    OnCalcResidualAndBuffersDebugStart  _OnCalcResidualAndBuffersDebugStart;
    OnCalcResidualAndBuffersDebugFinish _OnCalcResidualAndBuffersDebugFinish;
    OnSetSecondFrame                    _OnSetSecondFrame;
    OnTrackingFinishedDisplayResiduals  _OnTrackingFinishedDisplayResiduals;
    
    OnSignalConnectedType               _onSignalConnected;
private:
    //util::SE3TrackerSignals::OnSignalConnectedType _onSignalConnected;
    void OnSignalConnectedHandler(const util::SE3TrackerSignals* pSignals,const boost::signals2::signal_base*,bool connected);
};

}
