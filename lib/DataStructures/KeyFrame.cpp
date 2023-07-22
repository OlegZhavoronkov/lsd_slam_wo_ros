

#include "KeyFrame.h"
#include "Frame.h"


namespace lsd_slam {

  //== Static functions for creation
  KeyFrame::SharedPtr KeyFrame::Create( const Frame::SharedPtr &frame ) {
    KeyFrame::SharedPtr kf( new KeyFrame( frame ) );

    kf->depthMap()->initializeFromFrame();
    kf->syncDepthMapToFrame();

    return kf;
  }

  KeyFrame::SharedPtr KeyFrame::PropagateAndCreate( const KeyFrame::SharedPtr &other, const Frame::SharedPtr &frame ) {
    KeyFrame::SharedPtr kf( new KeyFrame( frame ) );

    float rescaleFactor = 1.0;
    kf->depthMap()->propagateFrom( other->depthMap(), rescaleFactor );
    kf->syncDepthMapToFrame();

    kf->frame()->pose->setThisToParent( sim3FromSE3( se3FromSim3( kf->frame()->pose->_thisToParent_raw ), rescaleFactor ) );

    return kf;
  }


  //=== Class functions ===

  KeyFrame::KeyFrame( const Frame::SharedPtr &frame )
    : numFramesTrackedOnThis(0),
      numMappedOnThis(0),
      numMappedOnThisTotal(0),
      _frame( frame ),
      _depthMap( new DepthMap( frame ) ),
      _trackingReference( new TrackingReference( frame ) )
    {
      _frame->setDepth( _depthMap );
    }


  void KeyFrame::updateDepthFrom( const Frame::SharedPtr &frame ) {

    assert(frame->hasTrackingParent());

  	if(  frame->trackingParent()->id() != id() ) {
  		LOGF(WARNING, "updating keyframe %d with frame %d, which was tracked on a different keyframe (%d).  While this should work, it is not recommended.",
  				id(),
          frame->id(),
  				frame->trackingParent()->id());
  	}

    if( !_depthMap->updateDepthFrom( frame ) ) {
      // TODO Handle error

      return;
    }

    syncDepthMapToFrame();

  	numMappedOnThis++;
  	numMappedOnThisTotal++;

  }

  void KeyFrame::syncDepthMapToFrame() {
    _frame->setDepth(_depthMap);

  }

  void KeyFrame::finalize() {
    depthMap()->finalize();

    syncDepthMapToFrame();
    frame()->calculateMeanInformation();
    //frame()->takeReActivationData(currentDepthMap);
  }



}
