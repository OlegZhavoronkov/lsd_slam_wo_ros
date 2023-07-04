
#include "libvideoio/Undistorter.h"
#ifdef HAS_YAML_CPP
#include <iostream>
#include <sstream>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace libvideoio
{


cv::Mat loadFromYaml( const YAML::Node &node, const std::string &name, int expectedHeight, int expectedWidth )
{
	auto kdata = node["data"];
	const int nElem = expectedWidth*expectedHeight;
	CHECK(kdata.size() == nElem ) << "Expected " << nElem << " row-major elements in " << name << " matrix";

	const int krows = node["rows"].as<int>(),
						kcols = node["cols"].as<int>();
	CHECK(krows == expectedHeight && kcols == expectedWidth) << "Expected Camera matrix to be " << expectedHeight << " x " << expectedWidth << ", got " << krows << " x " << kcols;

	std::vector<double> kVec;
	for( auto elem : kdata ) {
		kVec.push_back(elem.as<double>());
	}

	cv::Mat m( krows, kcols, CV_64F, kVec.data() );
	cv::Mat out;

	// Need to ensure a unique copy of the data is created..
	m.copyTo(out);
	return out;
}

	// Creates an OpenCVUndistorter by reading a ROS camera_info YAML File
	//
	// Uses yaml-cpp because OpenCV's YAML reader is a mess
OpenCVUndistorter *ROSUndistorterFactory::loadFromFile( const std::string &yamlFile, const std::shared_ptr<Undistorter> & wrap )
{
	bool valid = true;

	YAML::Node yaml = YAML::LoadFile(yamlFile);

	const int width = yaml["image_width"].as<int>();
	const int height = yaml["image_height"].as<int>();

  if( !yaml["camera_matrix"]) {
    LOG(FATAL) << "Unable to load camera matrix from " << yamlFile;
  }

	cv::Mat originalK( loadFromYaml( yaml["camera_matrix"], "Camera matrix", 3, 3 ) );
	cv::Mat projection( loadFromYaml( yaml["projection_matrix"], "Projection matrix", 3, 4 ) );
	cv::Mat distortion( loadFromYaml( yaml["distortion_coefficients"], "Distortion coefficient", 1, 5 ) );
	cv::Mat rectification( loadFromYaml( yaml["rectification_matrix"], "Rectification matrix", 3, 3 ) );

	if (valid)
	{
		return new OpenCVUndistorter( originalK, projection, rectification, distortion, ImageSize( width, height ), wrap);
	}

	return nullptr;
}

}
#endif