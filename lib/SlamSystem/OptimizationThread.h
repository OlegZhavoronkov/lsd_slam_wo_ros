

#pragma once

#include <mutex>
#include <memory>

#include "active_object/active.h"

#include "util/MovingAverage.h"
#include "util/ThreadMutexObject.h"

namespace lsd_slam {

	class KeyFrameGraph;
	class SlamSystem;

class OptimizationThread {
public:

	// TODO.  Don't like passing reference to Mutex.  Another way to do it?

	OptimizationThread( SlamSystem &system, bool threaded );
	~OptimizationThread();

	// == Public interfaces to kick off events ==
	void doNewConstraint( void );
	

	void doFinalOptimization( void );
	


	ThreadSynchronizer finalOptimizationComplete;

	// bool haveUnmergedOptimizationOffset( void ) { return _haveUnmergedOptimizationOffset; }
	// void clearUnmergedOptimizationOffset( void )
	// { if( _thread ) _thread->send( std::bind(&OptimizationThread::callbackClearUnmergedOptimizationOffset, this) ); }

	// std::mutex g2oGraphAccessMutex;
	// std::mutex newConstraintMutex;	// Not necessary because doNewConstraint is now imperative?

	MsRateAverage perf;

private:

	std::mutex optimizationThreadMutex;


	void idleImpl( void );
	void newConstraintImpl( void );
	void finalOptimizationImpl( void );
	// void callbackClearUnmergedOptimizationOffset( void ) { _haveUnmergedOptimizationOffset = false; }


	bool optimizationIteration(int itsPerTry, float minChange);

	// Function didn't appear to be called ... (?)
	// void OptimizationThread::optimizeGraph( void );

	// optimization merging. SET in Optimization, merged in Mapping.
	// bool _haveUnmergedOptimizationOffset;

	SlamSystem &_system;

	std::shared_ptr<active_object::ActiveIdle> _thread;

};


}
