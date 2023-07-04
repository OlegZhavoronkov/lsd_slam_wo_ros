
#include "libvideoio/ImageOutput.h"
#include "logger/LogFields.h"

#include <opencv2/highgui/highgui.hpp>

namespace libvideoio {


	ImageOutput::ImageOutput( const string &path )
		: _path( path ),
			_active( false )
	{
		if( !_path.empty() ) {
			LOG(INFO) << "Recording to directory " << _path.string();

			if( !is_directory( _path ) ) {
				LOG(WARNING) << "Making directory " << _path.string();
				create_directory( _path );
			}

			_active = true;
		}
	}

	void ImageOutput::registerField( logger::FieldHandle_t handle, const string &name )
	{
		if( handle >= 0 ) {
			_names[handle] = name;
			_count[handle] = 0;
		}
	}

	bool ImageOutput::write( logger::FieldHandle_t handle, const cv::Mat &img, int frame )
	{
		if( !_active ) return true;
		if( _names.count(handle) == 0 ) return  false;
		char buf[80];
		snprintf(buf, 79, "%s_%06d.png", _names[handle].c_str(), (frame < 0 ? _count[handle] : frame ) );
		fs::path imgPath( _path );
		imgPath /= buf;

		cv::imwrite( imgPath.string(), img );
		_count[handle]++;
		return true;
	}

}
