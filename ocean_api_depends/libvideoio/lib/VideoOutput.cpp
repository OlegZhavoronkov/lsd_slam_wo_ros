#include "g3log/g3log.hpp"

#include "libvideoio/VideoOutput.h"

namespace libvideoio {

	VideoOutput::VideoOutput( const std::string &filename, float fps, const std::string &fourcc )
	: _file( filename ),
	_active( false ),
	_writer(),
	_fps( fps ),
	_fourcc( fourcc )
	{

		if( !_file.empty() ) {
			LOG(INFO) << "Recording to video file " << _file.string();
			_active = true;
		}
	}

	// bool write( logger::FieldHandle_t handle, const Mat &img, int frame = -1 )
	bool VideoOutput::write( const cv::Mat &img )
	{
		if( !_active ) return true;

		// Eager-create the writer
		if( !_writer ) {
			LOG(INFO) << "Opening video at " << _fps << " fps with size " << img.cols << " x " << img.rows;

			const char *fcc = _fourcc.c_str();

			_writer.reset( new cv::VideoWriter(_file.string(), cv::VideoWriter::fourcc(fcc[0], fcc[1], fcc[2], fcc[3]), _fps, cv::Size(img.cols, img.rows)));

			LOG_IF( FATAL, _writer->isOpened() == false) << "Unable to open video writer.";
		}

		_writer->write( img );

		return true;
	}


}
