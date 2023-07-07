

#pragma once

#include <mutex>
#include <memory>

#include <shared_mutex>

#include "active_object/active.h"

#include "util/MovingAverage.h"
#include "util/ThreadMutexObject.h"

#include "DataStructures/KeyFrame.h"

#include "DepthEstimation/DepthMap.h"
#include "Tracking/TrackingReference.h"

#include "Tracking/Relocalizer.h"


namespace lsd_slam {

	class KeyFrameGraph;
	class SlamSystem;

class MappingThread {
public:

	MappingThread( SlamSystem &system, bool threaded );
	~MappingThread();

	//=== Callbacks into the thread ===
	// void pushDoIteration()
	// {
	//   //if( _thread ) _thread->send( std::bind( &MappingThread::doMappingIteration, this ));
	//   if( _thread ) {
	// 		_thread->send( std::bind( &MappingThread::doMappingIterationSet, this ));
	// 	} else {
	// 		doMappingIterationSet();
	// 	}
	// }
        /* REDUNDANT
	void pushUnmappedTrackedFrame( const Frame::SharedPtr &frame )
	{
		{
			std::lock_guard<std::mutex> lock(unmappedTrackedFramesMutex );
			unmappedTrackedFrames.push_back( frame );
		}

		if( _thread ) _thread->send( std::bind( &MappingThread::callbackUnmappedTrackedFrames, this ));
	}
        */

	void doMapSet( const KeyFrame::SharedPtr &kf, const ImageSet::SharedPtr &set)
	{
	    if( _thread )
				_thread->send( std::bind( &MappingThread::mapSetImpl, this, kf, set ));
			else
				mapSetImpl(kf, set);
	}

	void doMergeOptimizationUpdate( void )
	{
		optimizationUpdateMerged.reset();
		if( _thread ) _thread->send( std::bind( &MappingThread::mergeOptimizationOffsetImpl, this ));
	}

	// Create the first (uninitialized) keyframe
	void createFirstKeyFrame( const Frame::SharedPtr &frame );

	// Create subsequent keyframes by depth map propagation
	void doCreateNewKeyFrame( const KeyFrame::SharedPtr &keyframe, const Frame::SharedPtr &frame )
	{
		if( _thread )
			_thread->send( std::bind( &MappingThread::createNewKeyFrameImpl, this, keyframe, frame ));
		else
			createNewKeyFrameImpl( keyframe, frame );

	}

	// Used during re-localization
	Relocalizer relocalizer;

	ThreadSynchronizer optimizationUpdateMerged;


private:

	SlamSystem &_system;


	// == Thread callbacks ==
  void mapSetImpl( const KeyFrame::SharedPtr &kf, const ImageSet::SharedPtr &set );

	void createNewKeyFrameImpl( const KeyFrame::SharedPtr &keyframe, const Frame::SharedPtr &frame );

	// == Local functions ==

	void mergeOptimizationOffsetImpl();


	// == Local functions ==

	void addTimingSamples();

	void finishCurrentKeyframe();
	void discardCurrentKeyframe();

	void debugDisplayDepthMap();


	std::unique_ptr<active_object::Active> _thread;

};


}
