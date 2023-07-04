
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

}
