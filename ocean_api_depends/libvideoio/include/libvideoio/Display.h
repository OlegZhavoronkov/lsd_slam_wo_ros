#pragma once

#include "active_object/active.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace libvideoio {

using namespace cv;

class Display {
public:
	Display( bool doDisplay )
		: _doDisplay( doDisplay ),
			_displaySize( 640, 480 ),
			_active( active_object::Active::createActive() )
	{}

	void showLeft( Mat img )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onShowLeft, this, img )); }

	void showDepth( Mat img )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onShowDepth, this, img )); }

	void showRight( Mat img )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onShowRight, this, img )); }

	void showRawStereoYUV( Mat img )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onShowRawStereoYUV, this, img )); }

	void waitKey( int wk = 1 )
	{ if( _doDisplay ) _active->send( std::bind( &Display::onWaitKey, this, wk )); }

protected:

	void onShowLeft( const Mat &img );

	void onShowDepth( const Mat &img );

	void onShowRight( const Mat &img );

	void onShowRawStereoYUV( const Mat &img );

	void onWaitKey( unsigned int k )
	{
		cv::waitKey(k);
	}

	bool _doDisplay;
	cv::Size _displaySize;
	std::unique_ptr<active_object::Active> _active;
};

}
