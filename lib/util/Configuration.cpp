#include "Configuration.h"

#include "DataStructures/FrameMemory.h"

namespace lsd_slam {

  Configuration &Conf()
  {
    static Configuration TheInstance;
    return TheInstance;
  }

  Configuration::Configuration()
    : runRealTime( true ),
      doDepth( NO_STEREO ),
      stopOnFailedRead( true ),
      SLAMEnabled( true ),
      doKFReActivation( true ),
      doMapping( true ),
      continuousPCOutput( true ),

      debugDisplay( 0 ),

      onSceenInfoDisplay( true ),
      displayDepthMap( true ),
      dumpMap( false ),
      doFullReConstraintTrack( false ),
      print()
  {
  }


  const ImageSize &Configuration::setSlamImageSize( const ImageSize &sz ) {
    CHECK(sz.width%16 == 0 && sz.height%16 == 0) << "SLAM image dimensions must be multiples of 16! Please crop your images / video accordingly.";
    CHECK(sz.width!=0 && sz.height!=0 ) << "Height or width set to zero!";

    slamImageSize = sz;
    return slamImageSize;
  }


  //==
  Configuration::PrintConfiguration::PrintConfiguration()
    : threadingInfo( true ),
      memoryDebugInfo(false),
      trackingIterationInfo(false)
    {;}


}
