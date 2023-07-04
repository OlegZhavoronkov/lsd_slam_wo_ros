
#include <iostream>

#include <gtest/gtest.h>

#include "test_files.h"

#include "libvideoio/Undistorter.h"

using namespace libvideoio;

using namespace std;

TEST(PhotoscanXMLUndistorterFactory, Load_d2_camera) {

  std::shared_ptr<OpenCVUndistorter> undistorter( PhotoscanXMLUndistorterFactory::loadFromFile( PHOTOSCAN_XML ) );

  cout << "Original K:" << endl << undistorter->getOriginalK() << endl;
  cout << "Processed K:" << endl << undistorter->getK() << endl;

}
