
#include <string>
using namespace std;

#include <opencv2/opencv.hpp>

#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <filesystem>
namespace fs = std::filesystem;

#include <CLI/CLI.hpp>

#include <libg3logger/g3logger.h>

#include "libvideoio/ImageSource.h"
#include "libvideoio/Undistorter.h"

#include "logger/LogWriter.h"

#include "libvideoio/Display.h"
#include "libvideoio/ImageOutput.h"
#include "libvideoio/VideoOutput.h"

using namespace libvideoio; // New namespace

bool keepGoing = true;

void signal_handler( int sig )
{
	switch( sig ) {
		case SIGINT:
		keepGoing = false;
		break;
	}
}

using cv::Mat;

int main( int argc, char** argv )
{
	libg3logger::G3Logger logWorker( argv[0] );
  logWorker.logBanner();

	signal( SIGINT, signal_handler );


	CLI::App app{""};

	CLI11_PARSE(app, argc, argv);

	//
	// try {
	// 	TCLAP::CmdLine cmd("LSDRecorder", ' ', "0.1");
	//
	// 	TCLAP::ValueArg<std::string> resolutionArg("r","resolution","Input resolution: hd2k,hd1080,hd720,vga",false,"hd1080","", cmd);
	// 	TCLAP::ValueArg<float> fpsArg("f","fps","Input FPS, otherwise defaults to max FPS from input source",false,0.0,"", cmd);
	//
	// 	TCLAP::ValueArg<std::string> logInputArg("","log-input","Input Logger file",false,"","", cmd);
	//
	// 	TCLAP::ValueArg<std::string> loggerOutputArg("l","log-output","Output Logger filename",false,"","", cmd);
	// 	TCLAP::ValueArg<std::string> calibOutputArg("","calib-output","Output calibration file (from stereolabs SDK)",false,"","Calib filename", cmd);
	//
	// 	TCLAP::ValueArg<std::string> compressionArg("","compression","Use Google Snappy compression",false,"snappy","", cmd);
	//
	// 	TCLAP::ValueArg<std::string> imageOutputArg("","image-output","Save output to individual frames",false,"","dir", cmd);
	// 	TCLAP::ValueArg<std::string> videoOutputArg("","video-output","Save output to video",false,"","filename", cmd);
	//
	// 	TCLAP::ValueArg<int> skipArg("","skip","",false,1,"", cmd);
	// 	TCLAP::ValueArg<int> startAtArg("","start-at","",false,0,"", cmd);
	//
	//
	// 	TCLAP::ValueArg<std::string> statisticsOutputArg("","statistics-output","",false,"","", cmd);
	//
	// 	// TCLAP::SwitchArg noGuiSwitch("","no-gui","Don't show a GUI", cmd, false);
	//
	// 	TCLAP::SwitchArg depthSwitch("","depth","", cmd, false);
	// 	TCLAP::SwitchArg rightSwitch("","right","", cmd, false);
	//
	// 	TCLAP::SwitchArg guiSwitch("","display","", cmd, false);
	//
	//
	// 	TCLAP::ValueArg<int> durationArg("","duration","Duration",false,0,"seconds", cmd);
	//
	// 	cmd.parse(argc, argv );
	//
	// 	int compressLevel = logger::LogWriter::DefaultCompressLevel;
	// 	if( compressionArg.isSet() ) {
	// 		if( compressionArg.getValue() == "snappy" )
	// 			compressLevel = logger::LogWriter::SnappyCompressLevel;
	// 		else {
	// 			try {
	// 				compressLevel = std::stoi(compressionArg.getValue() );
	// 			} catch ( std::invalid_argument &e ) {
	// 				throw TCLAP::ArgException("Don't understand compression level.");
	// 			}
	//
	// 		}
	// 	}
	//
	// 	// Output validation
	// 	if( !videoOutputArg.isSet() && !imageOutputArg.isSet() && !loggerOutputArg.isSet() && !guiSwitch.isSet() ) {
	// 		LOG(WARNING) << "No output options set.";
	// 		exit(-1);
	// 	}
	//
	// 	Display display( guiSwitch.getValue() );
	// 	ImageOutput imageOutput( imageOutputArg.getValue() );
	//
	// 	ImageSource *dataSource = NULL;
	//
	//
	// 	if( logInputArg.isSet() ) {
	// 		LOG(INFO) << "Loading logger data from " << logInputArg.getValue();
	// 		dataSource = new LoggerSource( logInputArg.getValue() );
	//
	// 		LOG_IF(FATAL, depthSwitch.getValue() && !dataSource->hasDepth() ) << "Depth requested but log file doesn't have depth data.";
	// 		LOG_IF(FATAL, rightSwitch.getValue() && dataSource->numImages() < 2 ) << "Depth requested but log file doesn't have depth data.";
	//
	// 		if( calibOutputArg.isSet() )
	// 			LOG(WARNING) << "Can't create calibration file from a log file.";
	//
	// 	}
	//
	// 	int numFrames = dataSource->numFrames();
	// 	float fps = dataSource->fps();
	//
	// 	CHECK( fps >= 0 );
	//
	// 	logger::LogWriter logWriter( compressLevel );
	// 	logger::FieldHandle_t leftHandle = 0, rightHandle = 1, depthHandle = 2;
	// 	if( loggerOutputArg.isSet() ) {
	// 		cv::Size sz(  dataSource->imageSize().cvSize() );
	//
	// 		leftHandle = logWriter.registerField( "left", sz, logger::FIELD_BGRA_8C );
	// 		if( depthSwitch.getValue() ) depthHandle = logWriter.registerField( "depth", sz, logger::FIELD_DEPTH_32F );
	// 		if( rightSwitch.getValue() ) rightHandle = logWriter.registerField( "right", sz, logger::FIELD_BGRA_8C );
	//
	// 		if( !logWriter.open( loggerOutputArg.getValue() ) ) {
	// 			LOG(FATAL) << "Unable to open file " << loggerOutputArg.getValue() << " for logging.";
	// 		}
	// 	}
	//
	// 	imageOutput.registerField( leftHandle, "left" );
	// 	imageOutput.registerField( rightHandle, "right" );
	// 	imageOutput.registerField( depthHandle, "depth" );
	//
	// 	VideoOutput videoOutput( videoOutputArg.getValue(), fps > 0 ? fps : 30 );
	//
	// 	int dt_us = (fps > 0) ? (1e6/fps) : 0;
	// 	const float sleepFudge = 1.0;
	// 	dt_us *= sleepFudge;
	//
	// 	std::chrono::steady_clock::time_point start( std::chrono::steady_clock::now() );
	// 	int duration = durationArg.getValue();
	// 	std::chrono::steady_clock::time_point end( start + std::chrono::seconds( duration ) );
	//
	// 	if( duration > 0 ){
	// 		LOG(INFO) << "Will log for " << duration << " seconds or press CTRL-C to stop.";
	// 	} else {
	// 		LOG(INFO) << "Logging now, press CTRL-C to stop.";
	// 	}
	//
	// 	// Wait for the auto exposure and white balance
	// 	std::this_thread::sleep_for(std::chrono::seconds(1));
	//
	// 	int count = 0, skip = skipArg.getValue();
	// 	while( keepGoing ) {
	// 		if( count > 0 && (count % 100)==0 ) LOG(INFO) << count << " frames";
	//
	// 		std::chrono::steady_clock::time_point loopStart( std::chrono::steady_clock::now() );
	//
	// 		if( (duration > 0) && (loopStart > end) ) { keepGoing = false;  break; }
	//
	//
	// 			if( dataSource->grab() ) {
	//
	// 				if( count > startAtArg.getValue() ) {
	//
	// 				cv::Mat left;
	// 				dataSource->getImage( 0, left );
	//
	// 				imageOutput.write( leftHandle, left );
	// 				videoOutput.write( left );
	//
	// 				if( loggerOutputArg.isSet() ) {
	// 					logWriter.newFrame();
	// 					// This makes a copy of the data to send it to the compressor
	// 					logWriter.addField( leftHandle, left );
	// 				}
	//
	// 				if( count % skip == 0 ) {
	// 					display.showLeft( left );
	// 				}
	//
	// 				if( rightSwitch.getValue() ) {
	// 					cv::Mat right;
	// 					dataSource->getImage( 1, right );
	//
	// 					imageOutput.write( rightHandle, right );
	//
	// 					if( loggerOutputArg.isSet() ) {
	// 						logWriter.addField( rightHandle, right.data );
	// 					}
	//
	// 					if( count % skip == 0 )
	// 						display.showRight( right );
	//
	// 				}
	//
	// 				if( depthSwitch.getValue() ) {
	// 					cv::Mat depth;
	// 					dataSource->getDepth( depth );
	//
	// 					// Before normalization
	// 					if( loggerOutputArg.isSet() )
	// 						logWriter.addField( depthHandle, depth.data );
	//
	// 					// Normalize depth
	// 					double mn = 1.0, mx = 1.0;
	// 					minMaxLoc( depth, &mn, &mx );
	// 					Mat depthInt( depth.cols, depth.rows, CV_8UC1 ), depthNorm( depth.cols, depth.rows, depth.type() );
	// 					depthNorm = depth * 255 / mx;
	// 					depthNorm.convertTo( depthInt, CV_8UC1 );
	//
	// 					imageOutput.write( depthHandle, depthInt );
	//
	// 					if( count % skip == 0 )
	// 						display.showDepth( depth );
	// 					}
	//
	// 				if( loggerOutputArg.isSet() ) {
	// 					const bool doBlock = false; //( dt_us == 0 );
	// 					if( !logWriter.writeFrame( doBlock ) ) {
	// 						LOG(WARNING) << "Error while writing frame...";
	// 					}
	// 				}
	// 				}
	//
	// 			} else {
	// 				LOG(WARNING) << "Problem grabbing from camera.";
	// 			}
	//
	//
	// 		if( count % skip == 0 )
	// 			display.waitKey();
	//
	//
	// 		if( dt_us > 0 ) {
	// 			std::chrono::steady_clock::time_point sleepTarget( loopStart + std::chrono::microseconds( dt_us ) );
	// 			//if( std::chrono::steady_clock::now() < sleepTarget )
	// 			std::this_thread::sleep_until( sleepTarget );
	// 		}
	//
	//
	// 		count++;
	//
	// 		if( numFrames > 0 && count >= numFrames ) {
	// 			keepGoing = false;
	// 		}
	// 	}
	//
	//
	// 	LOG(INFO) << "Cleaning up...";
	// 	std::chrono::duration<float> dur( std::chrono::steady_clock::now()  - start );
	//
	// 	LOG(INFO) << "Recorded " << count << " frames in " <<   dur.count();
	// 	LOG(INFO) << " Average of " << (float)count / dur.count() << " FPS";
	//
	// 	std::string fileName;
	//
 	// 	if( loggerOutputArg.isSet() ) {
	// 		logWriter.close();
	// 		fileName = loggerOutputArg.getValue();
	// 	}
	//
	// 	if( !fileName.empty() ) {
	// 		unsigned int fileSize = fs::file_size( fs::path(fileName));
	// 		float fileSizeMB = float(fileSize) / (1024*1024);
	// 		LOG(INFO) << "Resulting file is " << fileSizeMB << " MB";
	// 		LOG(INFO) << "     " << fileSizeMB/dur.count() << " MB/sec";
	// 		LOG(INFO) << "     " << fileSizeMB/count << " MB/frame";
	//
	// 		if( statisticsOutputArg.isSet() ) {
	// 			ofstream out( statisticsOutputArg.getValue(), ios_base::out | ios_base::ate | ios_base::app );
	// 			if( out.is_open() ) {
	// 				//out << resolutionToString( zedResolution ) << "," << fps << "," << (guiSwitch.isSet() ? "display" : "") << "," << count << "," << dur.count() << ","
	// 				//		<< fileSizeMB << endl;
	// 			}
	// 		}
	// 	}
	//
	//
	//
	// 	if( dataSource ) delete dataSource;
	//
	// } catch (TCLAP::ArgException &e)  // catch any exceptions
	// {
	// 	LOG(WARNING) << "error: " << e.error() << " for arg " << e.argId();
	// 	exit(-1);
	// }



	return 0;
}
