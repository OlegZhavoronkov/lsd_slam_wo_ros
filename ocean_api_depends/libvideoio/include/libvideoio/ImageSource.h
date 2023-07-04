
#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#include <g3log/g3log.hpp>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;


#include "FileUtils.h"
#include "libvideoio/types/ImageSize.h"

#include "logger/LogReader.h"


namespace libvideoio {

using namespace cv;

class ImageSource {
public:
  ImageSource( void )
    : _fps( 0.0 ), _outputType(-1)
  {;}

  virtual ~ImageSource()
  {;}

  ImageSource( const ImageSource & ) = delete;
  ImageSource &operator=( const ImageSource & ) = delete;

  bool hasDepth( void ) const { return _hasDepth; }
  int numImages( void ) const { return _numImages; }

  virtual int numFrames( void ) const = 0;

  virtual ImageSize imageSize( void ) const = 0;

  virtual bool grab( void ) = 0;

  virtual int getRawImage( int i, cv::Mat &mat ) = 0;

  virtual int getImage( int i, cv::Mat &mat );
  virtual int getImage( cv::Mat &mat ) { return getImage(0, mat); };

  virtual void getDepth( cv::Mat &mat ) { return; }

  float fps( void ) const { return _fps; }
  void setFPS( float f ) { _fps = f; }

  void setOutputType( int type ) { _outputType = type; }

  virtual int cvtToRGB() { return -1; }
  virtual int cvtToGray() { return -1; }

protected:

  int _numImages;
  bool _hasDepth;
  float _fps;

  int _outputType;

};


class ImageFilesSource : public ImageSource {
public:
  ImageFilesSource( const std::vector<std::string> &paths )
    : _idx( -1 )
  {
    for( std::string pathStr : paths ) {
      fs::path p( pathStr );
      if( fs::is_directory( p ) )
        getdir( p, _paths );
      else if( fs::is_regular_file( p ) )
        _paths.push_back( p );

      _hasDepth = false;
      _numImages = 1;
    }

  }

  ImageFilesSource( const std::vector<fs::path> &paths )
    : _idx( -1 )
  {
    for( fs::path p : paths ) {
      if( fs::is_directory( p ) )
        getdir( p, _paths );
      else
        _paths.push_back(p);

      _hasDepth = false;
      _numImages = 1;
    }

  }

  virtual int numFrames( void ) const { return _paths.size(); }

  virtual bool grab( void )
  {
    ++_idx;

    if( _idx >= (int)_paths.size() ) return false;

    return true;
  }

  virtual int getRawImage( int i, cv::Mat &mat )
  {
    if( i != 0 ) return 0;

    if( _idx >= (int)_paths.size() ) return -1;

    mat = cv::imread( _paths[_idx].string(), cv::IMREAD_GRAYSCALE );
    return _idx;
  }

  virtual ImageSize imageSize( void ) const
  {
    return ImageSize( 1, 1 );
  }

protected:

  std::vector<fs::path> _paths;
  int _idx;

};

class LoggerSource : public ImageSource {
public:
  LoggerSource( const std::string &filename )
    : _reader( )
  {
    CHECK( fs::is_regular_file( fs::path(filename ))) << "Couldn't open log file \"" << filename << "\"";

    CHECK( _reader.open( filename ) ) << "Couldn't open log file \"" << filename << "\"";

    _leftHandle = _reader.findField("left");
    _rightHandle = _reader.findField("right");
    _depthHandle = _reader.findField("depth");

    CHECK( _leftHandle >= 0 ) << "Couldn't find left image";

    _numImages = (_rightHandle >= 0 ) ? 2 : 1;
    _hasDepth = (_depthHandle >= 0) ? true : false;
  }

  virtual int numFrames( void ) const { return _reader.getNumFrames(); }

  virtual bool grab( void )
  {
    return _reader.grab();
  }

  virtual int getRawImage( int i, cv::Mat &mat )
  {
    if( i < 0 || i >= _numImages )  return -1;

    if( i == 0 )
      mat = _reader.retrieve( _leftHandle );
    else if( i == 1 )
      mat = _reader.retrieve( _rightHandle );

    return 0;
  }

  virtual void getDepth( cv::Mat &mat ) {
    if( !_hasDepth ) return;

    mat = _reader.retrieve( _depthHandle );
  }

  virtual ImageSize imageSize( void ) const
  {
    return ImageSize( 0,0 );
  }

protected:

  logger::LogReader _reader;
  logger::FieldHandle_t _leftHandle, _rightHandle, _depthHandle;

};

class VideoSource : public ImageSource {
public:

  VideoSource( const std::string &path )
    : _path( path ),
      _capture( path )
  {
       _hasDepth = false;
       _numImages = 1;

       setFPS(  _capture.get(cv::CAP_PROP_FPS) );
  }

  ~VideoSource()
{
  if( _capture.isOpened() ) _capture.release();

}

  void skipTo( int frame )
  {
    _capture.set( cv::CAP_PROP_POS_FRAMES, frame );
  }

  virtual int numFrames( void ) const
  {
    cv::VideoCapture &vc( const_cast< cv::VideoCapture &>(_capture) );
    return vc.get(cv::CAP_PROP_FRAME_COUNT);
  }

  virtual int frameNum( void ) const
  {
    cv::VideoCapture &vc( const_cast< cv::VideoCapture &>(_capture) );
    return vc.get(cv::CAP_PROP_POS_FRAMES);
  }

  virtual bool grab( void )
  {
    _capture.grab();

    // ++_idx;
    //
    // if( _idx >= _paths.size() ) return false;
    //
    return true;
  }

  virtual int getRawImage( int i, cv::Mat &mat )
  {
    _capture.retrieve( mat, i );
    return 0;
  }

  virtual ImageSize imageSize( void ) const
  {
    return cvSize();
  }

  cv::Size cvSize( void ) const
  {
    cv::VideoCapture &vc( const_cast< cv::VideoCapture &>(_capture) );
    return cv::Size( vc.get(cv::CAP_PROP_FRAME_WIDTH),
     								 vc.get(cv::CAP_PROP_FRAME_HEIGHT) );
  }

  bool isOpened() const
  {
    return _capture.isOpened();
  }

protected:

  fs::path _path;
  cv::VideoCapture _capture;


};



}
