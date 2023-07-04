
#include <iostream>

#include <gtest/gtest.h>

#include "test_files.h"

#include "libvideoio/Undistorter.h"

using namespace libvideoio;

using namespace std;

TEST(PTAMUndistorterFactory, Load_d2_camera) {

  std::shared_ptr<OpenCVUndistorter> undistorter( PTAMUndistorterFactory::loadFromFile( PTAM_JSON ) );

  cout << "Original K:" << endl << undistorter->getOriginalK() << endl;
  cout << "Processed K:" << endl << undistorter->getK() << endl;

  // Compare it to the original PTAM format
  PTAMUndistorter legacy( PTAM_LEGACY );

  cout << "Legacy K:" << endl << legacy.getOriginalK() << endl;
  cout << "Processed K:" << endl << legacy.getK() << endl;

  // Ensure they are equivalent
  cv::Mat diffOriginalK = undistorter->getOriginalK() - legacy.getOriginalK();
  cv::Mat diffK = undistorter->getK() - legacy.getK();

  double origKNorm = cv::norm( undistorter->getOriginalK(), legacy.getOriginalK(), cv::NORM_L1 );
  double kNorm     = cv::norm( undistorter->getK(), legacy.getK(), cv::NORM_L1 );

  ASSERT_LT( origKNorm, 1e-3 ) << "Legacy and JSON PTAM undistorter camera matrices differ";
  ASSERT_LT( kNorm, 1e-3 ) << "Legacy and JSON PTAM undistorter camera matrices differ";

}
