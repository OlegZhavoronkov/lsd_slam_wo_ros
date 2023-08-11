

#pragma once

#include "active_object/active.h"

#include "util/Configuration.h"
#include "util/ThreadMutexObject.h"
#include "DataStructures/KeyFrame.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
#include "Tracking/SE3Tracker.h"
#include "Tracking/Sim3Tracker.h"
#include "Tracking/TrackingReference.h"

namespace lsd_slam {

	class SlamSystem;

	struct KFConstraintStruct;

class ConstraintSearchThread {
public:
	ConstraintSearchThread( SlamSystem &system, bool threaded );
	~ConstraintSearchThread();

	void doFullReConstraintSearch( void );
//	{ fullReConstraintSearchComplete.reset();
//		if( _thread ) _thread->send( std::bind( &ConstraintSearchThread::fullReconstraintSearchImpl, this )); }

	// Note in non-threaded mode, this does nothing!
	void doCheckNewKeyFrame( const KeyFrame::SharedPtr &keyframe );
	//{ if( _thread ) _thread->send( std::bind( &ConstraintSearchThread::checkNewKeyFrameImpl, this, keyframe )); }

	ThreadSynchronizer fullReConstraintSearchComplete;

	struct PerformanceData {
		MsRateAverage findConstraint;
	};

	PerformanceData perf() const { return _perf; }


private:

	SlamSystem &_system;

 	PerformanceData _perf;

	std::shared_ptr<Sim3Tracker>       constraintTracker;
	std::shared_ptr<SE3Tracker>        constraintSE3Tracker;

	int _failedToRetrack;

	//=== Callbacks ===
	void idleImpl( void );
	int  fullReconstraintSearchImpl( void );

	void checkNewKeyFrameImpl( const KeyFrame::SharedPtr &keyframe );

	//=== Internal functions ====
	int findConstraintsForNewKeyFrames(const KeyFrame::SharedPtr &newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH);

	std::shared_ptr<active_object::ActiveIdle> _thread;

	float tryTrackSim3(
			const KeyFrame::SharedPtr &kfA,
			const KeyFrame::SharedPtr &kfB,
			int lvlStart, int lvlEnd,
			bool useSSE,
			Sim3 &AtoB,
			Sim3 &BtoA,
			KFConstraintStruct* e1=nullptr,
			KFConstraintStruct* e2=nullptr);

	void testConstraint(
			const KeyFrame::SharedPtr &keyframe,
			const KeyFrame::SharedPtr &candidate,
			KFConstraintStruct*& e1_out, KFConstraintStruct*& e2_out,
			Sim3 candidateToFrame_initialEstimate,
			float strictness);

};


}
