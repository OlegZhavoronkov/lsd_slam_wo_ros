
#include "libvideoio/Display.h"

namespace libvideoio {

using namespace cv;


	void Display::onShowLeft( const Mat &img )
	{
		cv::Mat resized;
		cv::resize( img, resized, _displaySize );
		cv::imshow( "Left", resized );
	}

	void Display::onShowDepth( const Mat &img )
	{
		cv::Mat resized;
		cv::resize( img*255, resized, _displaySize );
		cv::imshow( "Display", resized );
	}

	void Display::onShowRight( const Mat &img )
	{
		cv::Mat resized;
		cv::resize( img, resized, _displaySize );
		cv::imshow( "Right", resized );
	}

	void Display::onShowRawStereoYUV( const Mat &img )
	{
		// Zed presents YUV data as 4 channels at {resolution} (e.g. 640x480)
		// despite actually showing both Left and Right (e.g. 1280x480)
		// This actually makes sense at YUV uses 4 bytes to show 2 pixels
		// presumably the channels are ordered [U, Y1, V, Y2]
		//
		// OpenCV expects two channels of [U,Y1], [V,Y2] at the actual
		// image resoluton (1280x480)
		// If the byte ordering (U,Y1,V,Y2) is the same, then a simple
		// reshape (2 channel, rows=0 means retain # of rows) should suffice

		cv::Mat leftRoi( img, cv::Rect(0,0, img.cols/2, img.rows ));
		cv::Mat leftBgr;
		cv::resize( leftRoi, leftBgr, _displaySize );
		cv::cvtColor( leftBgr, leftBgr, cv::COLOR_YUV2BGRA_YUYV );
		imshow( "RawLeft", leftBgr );
	}

}
