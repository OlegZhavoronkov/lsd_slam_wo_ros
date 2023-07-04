
#include <iostream>

#include <gtest/gtest.h>

#include "test_files.h"

#include "libvideoio/Undistorter.h"

using namespace libvideoio;

using namespace std;

TEST(ROSUndistorterFactory, Load_PointGray_Camera) {

  std::shared_ptr<OpenCVUndistorter> undistorter( ROSUndistorterFactory::loadFromFile( ROS_YAML ) );

  cout << "Original K:" << endl << undistorter->getOriginalK() << endl;
  cout << "Processed K:" << endl << undistorter->getK() << endl;

}
