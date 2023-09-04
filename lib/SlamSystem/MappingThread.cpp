
#include "MappingThread.h"

#include <g3log/g3log.hpp>

#include <shared_mutex>

#include "GlobalMapping/KeyFrameGraph.h"
#include "Tracking/TrackingReference.h"

#include "SlamSystem/ConstraintSearchThread.h"
#include "SlamSystem/TrackingThread.h"

#include "SlamSystem.h"
#include "util/settings.h"

namespace lsd_slam {

using active_object::Active;

MappingThread::MappingThread( SlamSystem &system, bool threaded )
	: relocalizer(),
		_system(system ),
		_thread( threaded ? Active::createActive() : NULL )
{
	LOG(INFO) << "Started Mapping thread";
}

MappingThread::~MappingThread()
{
    //{
    //    decltype(_thread) threadLocal=_thread;
    //    if(threadLocal && threadLocal->thd_.joinable())
    //    {
    //        threadLocal->thd_.join();
    //    }
    //}
    _thread.reset();
	//if( _thread ) 
    //{
    //    if
    //    delete _thread.release()
    //};
//	unmappedTrackedFrames.clear();
}


void MappingThread::mapSetImpl( const KeyFrame::SharedPtr &kf, const ImageSet::SharedPtr &set )
{
	// TODO.  Need code to handle when the queue is growing out of control...

	LOG(INFO) << "Mapping set " << set->id() << " onto KeyFrame " << kf->id();

	kf->updateDepthFrom( set->refFrame() );

	_system.updateDisplayDepthMap();
	_system.publishCurrentKeyframe();
}

void MappingThread::doCreateNewKeyFrame( const KeyFrame::SharedPtr &keyframe, const Frame::SharedPtr &frame )
{
    auto thread=_thread;
	if( thread )
    {
		thread->send( std::bind( &MappingThread::createNewKeyFrameImpl, this, keyframe, frame ));
    }
	else
    {
		createNewKeyFrameImpl( keyframe, frame );
    }
}

void MappingThread::createFirstKeyFrame( const Frame::SharedPtr &frame )
{
    LOGF(WARNING,"Making %d as first KeyFrame",frame->id());

	KeyFrame::SharedPtr kf( KeyFrame::Create( frame ) );
	_system.keyFrameGraph()->addKeyFrame( kf );
	_system.trackingThread()->doUseNewKeyFrame( kf );
}


void MappingThread::createNewKeyFrameImpl( const KeyFrame::SharedPtr &currentKeyFrame, const Frame::SharedPtr &frame )
{
    LOGF(WARNING,"Making %d as new keyframe with current keyFrame %d", frame->id(),currentKeyFrame->id());

	CHECK( frame->isTrackingParent( currentKeyFrame ) ) << "New keyframe does not track on current keyframe!";

	KeyFrame::SharedPtr kf( KeyFrame::PropagateAndCreate( currentKeyFrame, frame ) );
    if(!_depthMapUpdateKeyFrameSignal.empty())
    {
        kf->depthMap()->ConnectUpdateKeyFrameSignal([this](auto...args){this->_depthMapUpdateKeyFrameSignal(args...);});
    }
    if(!_depthMapDisplayUpdateKeyFrameSignal.empty())
    {
        kf->depthMap()->ConnectDisplayUpdateKeyFrameSignal([this](auto...args){this->_depthMapDisplayUpdateKeyFrameSignal(args...);});
    }
    if(!_depthMapPlotDepthSignal.empty())
    {
        kf->depthMap()->ConnectPlotDepthSignal([this](auto...args){this->_depthMapPlotDepthSignal(args...);});
    }
	_system.keyFrameGraph()->addKeyFrame( kf );
	_system.trackingThread()->doUseNewKeyFrame( kf );
	_system.constraintThread()->doCheckNewKeyFrame( kf );
}


// TODO.  Not updated post-move_current_keyframe
void MappingThread::mergeOptimizationOffsetImpl()
{
	LOG(DEBUG) << "Merging optimization offset";

	// lets us put the publishKeyframeGraph outside the mutex lock
	bool didUpdate = false;

	// if(_optThread->haveUnmergedOptimizationOffset())
	{
		std::shared_lock< std::shared_mutex > pose_lock(_system.poseConsistencyMutex);
		std::shared_lock< std::shared_mutex > kfLock( _system.keyFrameGraph()->keyframesAllMutex);

		for(unsigned int i=0;i<_system.keyFrameGraph()->keyframesAll.size(); i++)
			_system.keyFrameGraph()->keyframesAll[i]->frame()->pose->applyPoseGraphOptResult();

		// _optThread->clearUnmergedOptimizationOffset();

		didUpdate = true;
	}

	if ( didUpdate ) {
		_system.publishKeyframeGraph();
	}

	optimizationUpdateMerged.notify();
}

void MappingThread::doMapSet( const KeyFrame::SharedPtr &kf, const ImageSet::SharedPtr &set)
{
    auto thread= _thread;
    if( thread )
    {
		thread->send( std::bind( &MappingThread::mapSetImpl, this, kf, set ));
    }
	else
    {
        mapSetImpl(kf, set);
    }
			
}

void MappingThread::doMergeOptimizationUpdate(  )
{
	optimizationUpdateMerged.reset();
    auto thread= _thread;
	if( thread ) 
    {
        _thread->send( std::bind( &MappingThread::mergeOptimizationOffsetImpl, this ));
    }
    else
    {
        mergeOptimizationOffsetImpl();
    }
}


void MappingThread::pushDoIteration()
{
  
    if( _thread ) 
    {
		_thread->send( std::bind( &MappingThread::doMappingIteration, this ));
	}
    else 
    {
		doMappingIteration();
	}
}

bool MappingThread::doMappingIteration()
{
    auto& kg= *(_system.keyFrameGraph());
    std::unique_lock lock(kg.keyframesAllMutex);
    auto currentKF=_system.currentKeyFrame();
    kg.keyframesAll.push_back(currentKF);
    LOGF(WARNING,"keyframegraph contains %zu frames",kg.keyframesAll.size());
    return true;
}

//==== Actual meat ====

/*
bool MappingThread::doMappingIteration()
{

	// If there's no keyframe, then give up
	if( !(bool)_system.currentKeyFrame() ) {
		LOG(INFO) << "Nothing to map: no keyframe";
		return false;
	}

		// TODO:  Don't know what circumstances cause this to happens
	// if(!doMapping && currentKeyFrame()->idxInKeyframes < 0)
	// {
	// 	if(currentKeyFrame()->numMappedOnThisTotal >= MIN_NUM_MAPPED)
	// 		finishCurrentKeyframe();
	// 	else
	// 		discardCurrentKeyframe();
	//
	// 	map->invalidate();
	// 	LOGF(INFO, "Finished KF %d as Mapping got disabled!\n",currentKeyFrame()->id());
	//
	// 	changeKeyframe(true, true, 1.0f);
	// }

	//callbackMergeOptimizationOffset();
	//addTimingSamples();

	// if(dumpMap)
	// {
	// 	keyFrameGraph()->dumpMap(packagePath+"/save");
	// 	dumpMap = false;
	// }

        bool didSomething = true;

	// set mappingFrame
	if( _system.trackingThread()->trackingIsGood() )
	{
		// TODO:  Don't know under what circumstances doMapping = false
		// if(!doMapping)
		// {
		// 	//printf("tryToChange refframe, lastScore %f!\n", lastTrackingClosenessScore);
		// 	if(_system.trackingThread->lastTrackingClosenessScore > 1)
		// 		changeKeyframe(true, false, _system.trackingThread->lastTrackingClosenessScore * 0.75);
		//
		// 	if (displayDepthMap || depthMapScreenshotFlag)
		// 		debugDisplayDepthMap();
		//
		// 	return false;
		// }

		std::shared_ptr< Frame > frame( _newKeyFrame.const_ref() );
		if( frame ) {
			LOG(INFO) << "Set " << frame->id() << " as new key frame";
			finishCurrentKeyframe();
			_system.changeKeyframe(frame, false, true, 1.0f);

			_newKeyFrame().reset();
		} else {
			 didSomething = updateKeyframe();
		}

		_system.updateDisplayDepthMap();

		LOG(DEBUG) << "Tracking is good, updating key frame, " << (didSomething ? "DID" : "DIDN'T") << " do something";
	}

        //TODO have not seen this entered before?
	else
	{
		LOG(INFO) << "Tracking is bad";

		// invalidate map if it was valid.
		if(_system.depthMap()->isValid())
		{
			if( _system.currentKeyFrame()->numMappedOnThisTotal >= MIN_NUM_MAPPED)
				finishCurrentKeyframe();
			else
				discardCurrentKeyframe();

			_system.depthMap()->invalidate();
		}

		// start relocalizer if it isnt running already
		if(!relocalizer.isRunning)
			relocalizer.start(_system.keyFrameGraph()->keyframesAll);

		// did we find a frame to relocalize with?
		if(relocalizer.waitResult(50)) {

						// Frame* keyframe;
						// int succFrameID;
						// SE3 succFrameToKF_init;
						// std::shared_ptr<Frame> succFrame;
						//
						// relocalizer.stop();
						// relocalizer.getResult(keyframe, succFrame, succFrameID, succFrameToKF_init);

			relocalizer.stop();
			RelocalizerResult result( relocalizer.getResult() );

			_system.loadNewCurrentKeyframe(result.keyframe);

			_system.trackingThread->takeRelocalizeResult( result );
		}
	}

	return didSomething;
}
*/




} // namespace lsd_slam
