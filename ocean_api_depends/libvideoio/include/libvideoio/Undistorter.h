/**
* This file was once part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "nlohmann/json.hpp"
#include "g3log/g3log.hpp"

#include "libvideoio/types/Camera.h"
#include "libvideoio/types/ImageSize.h"

#include <tinyxml2.h>

namespace libvideoio {

class Undistorter
{
public:
  virtual ~Undistorter() {;}

  /**
   * Undistorts the given image and returns the result image.
   */
  virtual void undistort(const cv::Mat &image, cv::OutputArray result) const = 0;

  virtual void undistortDepth( const cv::Mat &depth, cv::OutputArray result) const { depth.copyTo( result ); }

  /**
   * Returns the intrinsic parameter matrix of the undistorted images.
   */
  virtual const cv::Mat getK() const = 0;

  virtual const Camera getCamera() const  { return Camera(getK()); }

  /**
   * Returns the intrinsic parameter matrix of the original images,
   */
  virtual const cv::Mat getOriginalK() const = 0;

  virtual ImageSize inputImageSize( void ) const = 0;
  virtual int getInputWidth() const { return inputImageSize().width; };
  virtual int getInputHeight() const { return inputImageSize().height; };

  virtual ImageSize outputImageSize( void ) const { return inputImageSize(); };
  virtual int getOutputWidth() const { return outputImageSize().width; };
  virtual int getOutputHeight() const { return outputImageSize().height; };

  /**
   * Returns if the undistorter was initialized successfully.
   */
  virtual bool isValid() const = 0;

  /**
   * Returns undistorter name, if given
   */
  const std::string &name() const           { return _name; }
  void setName( const std::string &name )   { _name = name; }

protected:

  Undistorter(const std::shared_ptr<Undistorter> &wrap  = nullptr )
    : _wrapped(wrap), _name("(undefined)") {;}

  std::shared_ptr<Undistorter> _wrapped;
  std::string _name;


};


// An OpenCVUndistorter uses the OpenCV map() function for
// undistortion and takes the standard 4- (or more) coefficient
// tangential distortion model
class OpenCVUndistorter : public Undistorter
{
public:

  OpenCVUndistorter( const cv::Mat &k,
                      const cv::Mat &distCoeff,
                      const ImageSize &origSize,
                      const std::shared_ptr<Undistorter> & wrap  = nullptr );

  OpenCVUndistorter( const cv::Mat &origK,
                      const cv::Mat &projection,
                      const cv::Mat &rectification,
                      const cv::Mat &distCoeff,
                      const ImageSize &origSize,
                      const std::shared_ptr<Undistorter> & wrap  = nullptr );

  virtual ~OpenCVUndistorter();

  /**
   * Undistorts the given image and returns the result image.
   */
  virtual void undistort(const cv::Mat &image, cv::OutputArray result) const;
  virtual void undistortDepth( const cv::Mat &depth, cv::OutputArray result) const { depth.copyTo( result ); }

  /**
   * Returns the intrinsic parameter matrix of the undistorted images.
   */
  const cv::Mat getK() const { return _K; }

  /**
   * Returns the intrinsic parameter matrix of the original images,
   */
  const cv::Mat getOriginalK() const { return _originalK; }

  virtual ImageSize inputImageSize() const  { return _inputSize; }
  virtual ImageSize outputImageSize() const  { return _outputSize; }

  const cv::Vec3d baseline() const { return _baseline; }

  /**
   * Returns if the undistorter was initialized successfully.
   */
  bool isValid() const   { return _valid; }

protected:

  cv::Mat _originalK, _K;
  cv::Mat _distCoeffs;

  cv::Vec3d _baseline;

  ImageSize _inputSize, _outputSize;
  cv::Mat _map1, _map2;

  /// true if the undistorter object is valid (has been initialized with
  /// a valid configuration)
  bool _valid;

};


class UndistorterFactory {
public:
  /**
   * This function attempts to auto-detect the calibration file type.
   * Creates and returns an Undistorter of the type used by the given
   * configuration file. If the format is not recognized, returns nullptr.
   */
  static Undistorter* getUndistorterFromFile(const std::string &configFilename, const std::shared_ptr<Undistorter> & wrap  = nullptr );

};

// Each of these input files can be mapped to the OpenCV distortion model
class PhotoscanXMLUndistorterFactory : public UndistorterFactory {
public:
  static OpenCVUndistorter *loadFromFile( const std::string &filename, const std::shared_ptr<Undistorter> & wrap  = nullptr );
  static OpenCVUndistorter *loadFromXML( tinyxml2::XMLDocument &doc, const std::string&filename = "", const std::shared_ptr<Undistorter> & wrap  = nullptr );
};

class PTAMUndistorterFactory : public UndistorterFactory {
public:
  static OpenCVUndistorter *loadFromFile( const std::string &filename, const std::shared_ptr<Undistorter> & wrap  = nullptr );
  static OpenCVUndistorter *loadFromJSON( const nlohmann::json &json, const std::shared_ptr<Undistorter> & wrap  = nullptr );
};

class OpenCVUndistorterFactory : public UndistorterFactory {
public:
  static OpenCVUndistorter *loadFromFile( const std::string &filename, const std::shared_ptr<Undistorter> & wrap  = nullptr );
};

#ifdef HAS_YAML_CPP
class ROSUndistorterFactory : public UndistorterFactory {
public:
  static OpenCVUndistorter *loadFromFile( const std::string &filename, const std::shared_ptr<Undistorter> & wrap  = nullptr );
};
#endif


//=== Legacy Undistorter classes ===



class PTAMUndistorter : public Undistorter
{
public:
  /**
   * Creates an Undistorter by reading the distortion parameters from a file.
   *
   * The file format is as follows:
   * d1 d2 d3 d4 d5
   * inputWidth inputHeight
   * crop / full / none
   * outputWidth outputHeight
   */
  PTAMUndistorter(const char* configFileName, const std::shared_ptr<Undistorter> & wrap  = nullptr );

  /**
   * Destructor.
   */
  ~PTAMUndistorter();

  PTAMUndistorter(const PTAMUndistorter&) = delete;
  PTAMUndistorter& operator=(const PTAMUndistorter&) = delete;

  /**
   * Undistorts the given image and returns the result image.
   */
  void undistort(const cv::Mat &image, cv::OutputArray result) const;

  /**
   * Returns the intrinsic parameter matrix of the undistorted images.
   */
  const cv::Mat getK() const;

  virtual const Camera getCamera() const;


  /**
   * Returns the intrinsic parameter matrix of the original images,
   */
  const cv::Mat getOriginalK() const;

  /**
   * Returns the width of the undistorted images in pixels.
   */
  int getOutputWidth() const;

  /**
   * Returns the height of the undistorted images in pixels.
   */
  int getOutputHeight() const;


  virtual ImageSize inputImageSize( void ) const
    { return ImageSize( getInputWidth(), getInputHeight() ); }

  /**
   * Returns the width of the input images in pixels.
   */
  int getInputWidth() const;

  /**
   * Returns the height of the input images in pixels.
   */
  int getInputHeight() const;


  /**
   * Returns if the undistorter was initialized successfully.
   */
  bool isValid() const;

private:
  cv::Mat K_;
  cv::Mat originalK_;

  float inputCalibration[5];
  float outputCalibration[5];
  int out_width, out_height;
  int in_width, in_height;
  float* remapX;
  float* remapY;


  /// Is true if the undistorter object is valid (has been initialized with
  /// a valid configuration)
  bool valid;

};


class ImageCropper : public Undistorter
{
public:
  /**
   * Creates an Undistorter by reading the distortion parameters from a file.
   *
   * The file format is as follows:
   * d1 d2 d3 d4 d5
   * inputWidth inputHeight
   * crop / full / none
   * outputWidth outputHeight
   */
  ImageCropper( int width, int height, int offsetX = 0, int offsetY = 0,
                const std::shared_ptr<Undistorter> & wrap  = nullptr )
                : Undistorter(wrap),
                 _offsetX(offsetX), _offsetY(offsetY),
                 _width(width),      _height(height)
                {;}

  /**
   * Destructor.
   */
  virtual ~ImageCropper()
  {;}

  ImageCropper(const ImageCropper&) = delete;
  ImageCropper& operator=(const ImageCropper&) = delete;

  /**
   * Undistorts the given image and returns the result image.
   */
  virtual void undistort(const cv::Mat &image, cv::OutputArray result) const
  {
    cv::Mat intermediate(image);
    if( _wrapped ) {
      _wrapped->undistort( image, intermediate );
    }

    cv::Mat roi( intermediate, cv::Rect( _offsetX, _offsetY, _width, _height ) );
    LOG(WARNING) << "Cropping to " << _width << " x " << _height;
    // cv::imshow("roi",roi);
    // cv::waitKey(10);
    result.assign( roi );
  }

  /**
   * Returns the intrinsic parameter matrix of the undistorted images.
   */
  //const cv::Mat getK() const;

  virtual const cv::Mat getK() const {
    if( _wrapped ) return _wrapped->getK();

    return cv::Mat::eye(3,3, CV_32F );
  }


  /**
   * Returns the intrinsic parameter matrix of the original images,
   */
  const cv::Mat getOriginalK() const { return getK(); }

  virtual ImageSize outputImageSize( void ) const
  { return ImageSize( _width, _height ); };

  virtual ImageSize inputImageSize( void ) const
    { if( _wrapped ) return _wrapped->outputImageSize();
      return ImageSize( _width, _height ); }

  /**
   * Returns the width of the input images in pixels.
   */
  int getInputWidth() const {
    if( _wrapped ) return _wrapped->getInputWidth();

    return _width;
  }

  /**
   * Returns the height of the input images in pixels.
   */
  int getInputHeight() const {
    if( _wrapped ) return _wrapped->getInputHeight();
    return _width;
  }


  /**
   * Returns if the undistorter was initialized successfully.
   */
  bool isValid() const { return true; }

private:


  int _offsetX, _offsetY;
  int _width, _height;

};


class ImageResizer : public Undistorter
{
public:
  /**
   * Creates an Undistorter by reading the distortion parameters from a file.
   *
   * The file format is as follows:
   * d1 d2 d3 d4 d5
   * inputWidth inputHeight
   * crop / full / none
   * outputWidth outputHeight
   */
  ImageResizer( int width, int height,
                const std::shared_ptr<Undistorter> & wrap  = nullptr )
                : Undistorter(wrap),
                 _width(width), _height(height)
                {;}

  /**
   * Destructor.
   */
  virtual ~ImageResizer()
  {;}

  ImageResizer(const ImageResizer&) = delete;
  ImageResizer& operator=(const ImageResizer&) = delete;

  /**
   * Undistorts the given image and returns the result image.
   */
  virtual void undistort(const cv::Mat &image, cv::OutputArray result) const
  {
    cv::Mat intermediate(image);
    if( _wrapped ) {
      _wrapped->undistort( image, intermediate );
    }

    cv::Mat shrunk;
    cv::resize(intermediate, shrunk, cv::Size( _width, _height ));
    LOG(INFO) << "Shrinking to " << _width << " x " << _height;
    // cv::imshow("Intermediate", intermediate);
    // cv::imshow("shrunk",shrunk);
    // cv::waitKey(10);
    result.assign( shrunk );
  }

  /**
   * Returns the intrinsic parameter matrix of the undistorted images.
   */
  //const cv::Mat getK() const;

  virtual const cv::Mat getK() const {
    if( _wrapped ) return _wrapped->getK();

    return cv::Mat::eye(3,3, CV_32F );
  }


  /**
   * Returns the intrinsic parameter matrix of the original images,
   */
  const cv::Mat getOriginalK() const { return getK(); }

  virtual ImageSize outputImageSize( void ) const
  { return ImageSize( _width, _height ); };

  virtual ImageSize inputImageSize( void ) const
    { if( _wrapped ) return _wrapped->outputImageSize();
      return ImageSize( _width, _height ); }

  /**
   * Returns the width of the input images in pixels.
   */
  int getInputWidth() const {
    if( _wrapped ) return _wrapped->getInputWidth();

    return _width;
  }

  /**
   * Returns the height of the input images in pixels.
   */
  int getInputHeight() const {
    if( _wrapped ) return _wrapped->getInputHeight();
    return _width;
  }


  /**
   * Returns if the undistorter was initialized successfully.
   */
  bool isValid() const { return true; }

private:


  int _width, _height;

};



// class UndistorterOpenCV : public Undistorter
// {
// public:
//   /**
//    * Creates an Undistorter by reading the distortion parameters from a file.
//    *
//    * The file format is as follows:
//    * fx fy cx cy d1 d2 d3 d4 d5 d6
//    * inputWidth inputHeight
//    * crop / full / none
//    * outputWidth outputHeight
//    */
//   UndistorterOpenCV(const char* configFileName);
//
//   /**
//    * Destructor.
//    */
//   ~UndistorterOpenCV();
//
//   UndistorterOpenCV(const UndistorterOpenCV&) = delete;
//   UndistorterOpenCV& operator=(const UndistorterOpenCV&) = delete;
//
//   /**
//    * Undistorts the given image and returns the result image.
//    */
//   void undistort(const cv::Mat &image, cv::OutputArray result) const;
//
//   /**
//    * Returns the intrinsic parameter matrix of the undistorted images.
//    */
//   const cv::Mat getK() const;
//
//   virtual const Camera getCamera() const;
//
//   /**
//    * Returns the intrinsic parameter matrix of the original images,
//    */
//   const cv::Mat getOriginalK() const;
//
//   /**
//    * Returns the width of the undistorted images in pixels.
//    */
//   int getOutputWidth() const;
//
//   /**
//    * Returns the height of the undistorted images in pixels.
//    */
//   int getOutputHeight() const;
//
//
//   /**
//    * Returns the width of the input images in pixels.
//    */
//   int getInputWidth() const;
//
//   /**
//    * Returns the height of the input images in pixels.
//    */
//   int getInputHeight() const;
//
//   /**
//    * Returns if the undistorter was initialized successfully.
//    */
//   bool isValid() const;
//
// private:
//   cv::Mat K_;
//   cv::Mat originalK_;
//
//   float inputCalibration[10];
//   float outputCalibration;
//   int out_width, out_height;
//   int in_width, in_height;
//   cv::Mat map1, map2;
//
//   /// Is true if the undistorter object is valid (has been initialized with
//   /// a valid configuration)
//   bool valid;
// };
//
//
// class UndistorterLogger : public Undistorter
// {
// public:
//   /**
//    * Creates an Undistorter by reading the distortion parameters from a file.
//    *
//    * The file format is as follows:
//    * fx fy cx cy
//    * inputWidth inputHeight
//    * cropWidth cropHeight
//    * outputWidth outputHeight
//    */
//   UndistorterLogger(const char* configFileName);
//
//   /**
//    * Destructor.
//    */
//   ~UndistorterLogger();
//
//   UndistorterLogger(const UndistorterLogger&) = delete;
//   UndistorterLogger& operator=(const UndistorterLogger&) = delete;
//
//   void undistort(const cv::Mat &image, cv::OutputArray result) const;
//   virtual void undistortDepth( const cv::Mat &depth, cv::OutputArray result) const;
//
//
//   const cv::Mat getK() const;
//   virtual const Camera getCamera() const;
//   virtual const Camera getOriginalCamera( void ) const { return _originalCamera; }
//
//
//   const cv::Mat getOriginalK() const;
//   int getOutputWidth() const            { return _finalSize.width; }
//   int getOutputHeight() const            { return _finalSize.height; }
//   int getInputWidth() const              { return _inputSize.width; }
//   int getInputHeight() const            { return _inputSize.height; }
//
//
//   bool isValid() const { return _valid; }
//
// protected:
//
//   UndistorterLogger( const ImageSize &inputSize, const Camera &cam  );
//
//   ImageSize _inputSize, _cropSize, _finalSize;
//   Camera _originalCamera;
//
//   bool _valid;
// };

}
