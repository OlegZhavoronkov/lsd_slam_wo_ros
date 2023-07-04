#pragma once

#include <string>
#include <map>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <opencv2/core/core.hpp>

#include "logger/LogFields.h" 

namespace libvideoio {

using namespace std;

class ImageOutput {
public:
	ImageOutput( const string &path );

	void registerField( logger::FieldHandle_t handle, const string &name );

	bool write( logger::FieldHandle_t handle, const cv::Mat &img, int frame = -1 );

protected:

	fs::path _path;
	bool _active;
	std::map< logger::FieldHandle_t, unsigned int > _count;

	std::map< logger::FieldHandle_t, std::string > _names;
};


}
