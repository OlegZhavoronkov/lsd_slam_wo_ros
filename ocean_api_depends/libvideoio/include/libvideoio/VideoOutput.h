#pragma once

#include <string>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace libvideoio {

class VideoOutput {
public:
	VideoOutput( const std::string &filename, float fps, const std::string &fourcc = "AVC1" );
	bool write( const cv::Mat &img );

	bool isActive( void ) const { return _active; }

protected:

	fs::path _file;
	bool _active;

	std::unique_ptr< cv::VideoWriter > _writer;
	float _fps;

	std::string _fourcc;
};

}
