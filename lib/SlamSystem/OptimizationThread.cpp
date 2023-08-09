
#include "OptimizationThread.h"

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"

#include <g3log/g3log.hpp>

#include "SlamSystem.h"
#include "MappingThread.h"

namespace lsd_slam {

using active_object::ActiveIdle;

const auto optimizationDt = std::chrono::milliseconds(2000);

OptimizationThread::OptimizationThread( SlamSystem &system, bool threaded )
	: //_haveUnmergedOptimizationOffset( false ),
		_system( system ),
		_thread( threaded ? ActiveIdle::createActiveIdle( std::bind( &OptimizationThread::idleImpl, this ), optimizationDt ).release() : NULL  )
{
	LOG(INFO) << "Started optimization thread";
}

OptimizationThread::~OptimizationThread()
{
	if( _thread ) 
    {
        _thread.reset();
    }
	LOG(INFO) << "Exited optimization thread";
}


//===== Callbacks for ActiveObject ======

void OptimizationThread::idleImpl( void )
{
	LOG(DEBUG) << "Running short optimization";

	//!!TODO.  Why does this happen in mapThread?
	while(optimizationIteration(5, 0.02)) { ; } //_system.mapThread()->doMergeOptimizationUpdate(); }
}

void OptimizationThread::newConstraintImpl( void )
{
	LOG(DEBUG) << "Running short optimization";

	_system.keyFrameGraph()->addElementsFromBuffer();

	//!!TODO.  Why does this happen in mapThread?
	while(optimizationIteration(5, 0.02)) { ; } //_system._mapThread->doMergeOptimizationUpdate(); }
}

void OptimizationThread::doFinalOptimization( void )
{ 
    finalOptimizationComplete.reset();
    auto thread= _thread;
	if( thread ) 
    {
        _thread->send( std::bind(&OptimizationThread::finalOptimizationImpl, this) ); 
    }
    else
    {
        finalOptimizationImpl();
    }
}

void OptimizationThread::doNewConstraint( void )
{
    auto thread = _thread;
    if( thread ) 
    {
        _thread->send( std::bind(&OptimizationThread::newConstraintImpl, this) ); 
    }
    else
    {
        newConstraintImpl();
    }
}

void OptimizationThread::finalOptimizationImpl( void )
{
	LOG(INFO) << "Running final optimization!";
	optimizationIteration(50, 0.001);

	//!!TODO.  Why does this happen in mapThread?
	_system.mapThread()->doMergeOptimizationUpdate();
	finalOptimizationComplete.notify();
}


//=== Actual meat ====





bool OptimizationThread::optimizationIteration(int itsPerTry, float minChange)
{
	Timer timer;

	// std::lock_guard< std::mutex > lock(g2oGraphAccessMutex);

	// Do the optimization. This can take quite some time!
	int its = _system.keyFrameGraph()->optimize(itsPerTry);

	// save the optimization result.
	_system.poseConsistencyMutex.lock_shared();
	_system.keyFrameGraph()->keyframesAllMutex.lock_shared();
	float maxChange = 0;
	float sumChange = 0;
	float sum = 0;
	for(size_t i=0;i<_system.keyFrameGraph()->keyframesAll.size(); i++)
	{
		// set edge error sum to zero
		_system.keyFrameGraph()->keyframesAll[i]->frame()->edgeErrorSum = 0;
		_system.keyFrameGraph()->keyframesAll[i]->frame()->edgesNum = 0;

		if(!_system.keyFrameGraph()->keyframesAll[i]->frame()->pose->isInGraph) continue;



		// get change from last optimization
		Sim3 a = _system.keyFrameGraph()->keyframesAll[i]->frame()->pose->graphVertex->estimate();
		Sim3 b = _system.keyFrameGraph()->keyframesAll[i]->frame()->getCamToWorld();
		Sophus::Vector7f diff = (a*b.inverse()).log().cast<float>();


		for(int j=0;j<7;j++)
		{
			float d = fabsf((float)(diff[j]));
			if(d > maxChange) maxChange = d;
			sumChange += d;
		}
		sum +=7;

		// set change
		_system.keyFrameGraph()->keyframesAll[i]->frame()->pose->setPoseGraphOptResult(
				_system.keyFrameGraph()->keyframesAll[i]->frame()->pose->graphVertex->estimate());

		// add error
		for(auto edge : _system.keyFrameGraph()->keyframesAll[i]->frame()->pose->graphVertex->edges())
		{
			_system.keyFrameGraph()->keyframesAll[i]->frame()->edgeErrorSum += ((EdgeSim3*)(edge))->chi2();
			_system.keyFrameGraph()->keyframesAll[i]->frame()->edgesNum++;
		}
	}

	_system.keyFrameGraph()->keyframesAllMutex.unlock_shared();
	_system.poseConsistencyMutex.unlock_shared();

	LOGF_IF(DEBUG, Conf().print.optimizationInfo,
					"did %d optimization iterations. Max Pose Parameter Change: %f; avgChange: %f. %s\n",
					its, maxChange, sumChange / sum,
					maxChange > minChange && its == itsPerTry ? "continue optimizing":"Waiting for addition to graph.");

	perf.update( timer );

	return maxChange > minChange && its == itsPerTry;
}

// void OptimizationThread::optimizeGraph( void )
// {
// 	std::unique_lock<std::mutex> g2oLock(g2oGraphAccessMutex);
// 	keyFrameGraph()->optimize(1000);
// 	g2oLock.unlock();
// 	mergeOptimizationOffset();
// }



}
