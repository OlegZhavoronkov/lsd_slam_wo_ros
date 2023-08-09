
#ifdef WITH_TINYXML
#include "libvideoio/Undistorter.h"

#include <tinyxml2.h>

#include <sstream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace libvideoio
{

  using namespace tinyxml2;

  // Sample input:
  //
  // <?xml version="1.0" encoding="UTF-8"?>
  // <calibration>
  //   <projection>frame</projection>
  //   <width>1920</width>
  //   <height>1080</height>
  //   <f>1430.15016509976</f>
  //   <cx>-24.1342060175983</cx>
  //   <cy>17.5452248329602</cy>
  //   <b2>-4.25145544487403</b2>
  //   <k1>-0.118322055927498</k1>
  //   <k2>0.293632083518507</k2>
  //   <p1>0.000330080086304322</p1>
  //   <p2>0.00182154893322038</p2>
  //   <date>2017-11-14T23:30:12Z</date>
  // </calibration>

  // From the Photoscan Manual:
  // (X,Y,Z) == point coords in local camera coord system
  //
  // x = X / Z
  // y = Y / Z
  // r = sqrt( x^2 + y^2 )
  //
  // x' = x(1 + K1r^2 + K2r^4 + K3r^6 + K4r^8 ) + (P1(r +2x ) + 2P2xy)(1 + P3r + P4r )
  // y' = y(1 + K1r^2 + K2r^4 + K3r^6 + K4r^8 ) + (P2(r +2y ) + 2P1xy)(1 + P3r + P4r )
  //
  // u = w * 0.5 + cx + x'f + x'B1 + y'B2
  // v = h * 0.5 + cy + y'f
  // (u,v) point coord in image coord system (in pixels)
  //
  // Interestingly, they seem to reverse p1 and p2 relative to OpenCV

  // For reference, OpenCV does:
  //
  // x' = X / W
  // y' = Y / W
  //
  // x'' = x'(1 + K1r^2 + K2r^4 + K3r^6 )/(1 + K4r^2 + K5r^4 + K6r^6) + 2p1x'y' + p2(r^2 + 2x'^2) + s1r^4 + s2r^4
  // y'' = y'(1 + K1r^2 + K2r^4 + K3r^6 )/(1 + K4r^2 + K5r^4 + K6r^6) + p1(r^2 + 2y'^2) + 2p2x'y' + s3r^2 + s4r^4
  //
  // u = x''fx + c_x
  // v = y''fy + c_y


  bool readDoubleFromXML( const XMLNode *parent, std::string name, double &value, double def = 0.0 )
  {
    auto child = parent->FirstChildElement(name.c_str());
    if( child ) {
      value = atof(child->GetText());
      return true;
    }

    value = def;
    return false;
  }

  bool readIntFromXML( const XMLNode *parent, std::string name, int &value, int def = 0 )
  {
    auto child = parent->FirstChildElement(name.c_str());
    if( child ) {
      value = atoi(child->GetText());
      return true;
    }

    value = def;
    return false;
  }

  OpenCVUndistorter *PhotoscanXMLUndistorterFactory::loadFromFile(const std::string &configFileName, const std::shared_ptr<Undistorter> & wrap )
  {
    XMLDocument doc;
    doc.LoadFile( configFileName.c_str() );

    return loadFromXML( doc, configFileName, wrap );
  }

  OpenCVUndistorter *PhotoscanXMLUndistorterFactory::loadFromXML( XMLDocument &doc, const std::string &configFileName, const std::shared_ptr<Undistorter> & wrap )
  {
    bool valid = true;
    auto topNode = doc.FirstChildElement( "calibration" );

    if( !topNode ) {
      LOG(WARNING) << "Unable to find top-level element <\"calibration\"> in calibration file " << configFileName;
      valid = false;
      return nullptr;
    }

    int width, height;
    valid = readIntFromXML(topNode, "width", width );
    valid = readIntFromXML(topNode, "height", height );

    double f=1.0, cx=0.0, cy=0.0;

    valid = readDoubleFromXML(topNode, "f", f);
    valid = readDoubleFromXML(topNode, "cx", cx);
    valid = readDoubleFromXML(topNode, "cy", cy);

    double b1 = 0.0, b2 = 0.0;
    readDoubleFromXML( topNode, "b1", b1 );
    readDoubleFromXML( topNode, "b2", b2 );

    cv::Mat originalK( cv::Mat(3, 3, CV_64F, cv::Scalar(0)) );

    originalK.at<double>(0, 0) = f+b1;
    originalK.at<double>(1, 1) = f;
    originalK.at<double>(2, 2) = 1;

    originalK.at<double>(0, 1) = b2;

    originalK.at<double>(0, 2) = cx + width/2.0;
    originalK.at<double>(1, 2) = cy + height/2.0;

    // OpenCV dist coeffs
    //    (k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4[,τx,τy]]]])

    cv::Mat distCoeffs( cv::Mat(1,5, CV_64F, cv::Scalar(0)) );
    double val;
    if( readDoubleFromXML( topNode, "k1", val ) ) {
      distCoeffs.at<double>(0,0) = val;
    }

    if( readDoubleFromXML( topNode, "k2", val ) ) {
      distCoeffs.at<double>(0,1) = val;
    }

    if( readDoubleFromXML( topNode, "k3", val ) ) {
      distCoeffs.at<double>(0,4) = val;
    }

    // Are the meaning of p1 and p2 really swapped rel to OpenCV?
    if (readDoubleFromXML( topNode, "p1", val ) ) {
      distCoeffs.at<double>(0,3) = val;
    }

    if (readDoubleFromXML( topNode, "p2", val ) ) {
      distCoeffs.at<double>(0,2) = val;
    }

    if (valid) {
      return new OpenCVUndistorter( originalK, distCoeffs, ImageSize(width,height), wrap );
    }

    // Else fail
    return nullptr;
  }




}
#endif