
#include <iostream>

#include <gtest/gtest.h>

#include "test_files.h"
#include "libvideoio/Undistorter.h"

using namespace libvideoio;
using namespace std;

namespace {

TEST( UndistorterFactory, LoadPTAMJSON ) {

  std::shared_ptr<Undistorter> undistorter( UndistorterFactory::getUndistorterFromFile( PTAM_JSON ) );

  ASSERT_TRUE( (bool)undistorter );

  // cout << "Original K:" << endl << undistorter->getOriginalK() << endl;
  // cout << "Processed K:" << endl << undistorter->getK() << endl;
}

TEST( UndistorterFactory, LoadPhotoscanXML ) {

  std::shared_ptr<Undistorter> undistorter( UndistorterFactory::getUndistorterFromFile( PHOTOSCAN_XML ) );
  ASSERT_TRUE( (bool)undistorter );

  // cout << "Original K:" << endl << undistorter->getOriginalK() << endl;
  // cout << "Processed K:" << endl << undistorter->getK() << endl;
}

TEST( UndistorterFactory, LoadROSYaml ) {

  std::shared_ptr<Undistorter> undistorter( UndistorterFactory::getUndistorterFromFile( ROS_YAML ) );
  ASSERT_TRUE( (bool)undistorter );

  // cout << "Original K:" << endl << undistorter->getOriginalK() << endl;
  // cout << "Processed K:" << endl << undistorter->getK() << endl;
}

}
