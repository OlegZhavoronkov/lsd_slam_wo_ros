
#include "libvideoio/Undistorter.h"

#include <sstream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace libvideoio
{

	// 	/**
	// 	 * Creates an Undistorter by reading the distortion parameters from a file.
	// 	 *
	// 	 * The file format is as follows:
	// 	 * fx fy cx cy d1 d2 d3 d4 d5 d6
	// 	 * inputWidth inputHeight
	// 	 * crop / full / none
	// 	 * outputWidth outputHeight
	// 	 */
OpenCVUndistorter *OpenCVUndistorterFactory::loadFromFile( const std::string &configFileName, const std::shared_ptr<Undistorter> & wrap )
{
	bool valid = true;

	// read parameters
	std::ifstream infile(configFileName);
	assert(infile.good());

	std::string l1, l2, l3, l4;

	std::getline(infile,l1);
	std::getline(infile,l2);
	std::getline(infile,l3);
	std::getline(infile,l4);

	float inputCalibration[10];
	[[maybe_unused]] /*using reult of it is commented out for now*/ float outputCalibration;
	int out_width, out_height;
	int in_width, in_height;

	// l1 & l2
	if(std::sscanf(l1.c_str(), "%f %f %f %f %f %f %f %f",
		&inputCalibration[0], &inputCalibration[1], &inputCalibration[2], &inputCalibration[3], &inputCalibration[4],
		&inputCalibration[5], &inputCalibration[6], &inputCalibration[7]
  				) == 8 &&
			std::sscanf(l2.c_str(), "%d %d", &in_width, &in_height) == 2)
	{
		printf("Input resolution: %d %d\n",in_width, in_height);
		printf("In: %f %f %f %f %f %f %f %f\n",
				inputCalibration[0], inputCalibration[1], inputCalibration[2], inputCalibration[3], inputCalibration[4],
				inputCalibration[5], inputCalibration[6], inputCalibration[7]);
	}
	else
	{
		printf("Failed to read camera calibration (invalid format?)\nCalibration file: %s\n", configFileName.c_str());
		valid = false;
	}

	// l3
	if(l3 == "crop")
	{
		outputCalibration = -1;
		printf("Out: Crop\n");
	}
	else if(l3 == "full")
	{
		outputCalibration = -2;
		printf("Out: Full\n");
	}
	else if(l3 == "none")
	{
		printf("NO RECTIFICATION\n");
		valid = false;
	}
	else
	{
		printf("Out: Failed to Read Output pars... not rectifying.\n");
		valid = false;
	}

	// l4
	if(std::sscanf(l4.c_str(), "%d %d", &out_width, &out_height) == 2)
	{
		printf("Output resolution: %d %d\n", out_width, out_height);
	}
	else
	{
		printf("Out: Failed to Read Output resolution... not rectifying.\n");
		valid = false;
	}

	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_32F);
	for (int i = 0; i < 4; ++ i)
		distCoeffs.at<float>(i, 0) = inputCalibration[4 + i];

	if(inputCalibration[2] < 1.0f)
	{
		printf("WARNING: cx = %f < 1, which should not be the case for normal cameras.!\n", inputCalibration[2]);
		printf("Possibly this is due to a recent change in the calibration file format, please see the README.md.\n");

		inputCalibration[0] *= in_width;
		inputCalibration[2] *= in_width;
		inputCalibration[1] *= in_height;
		inputCalibration[3] *= in_height;

		printf("auto-changing calibration file to fx=%f, fy=%f, cx=%f, cy=%f\n",
			inputCalibration[0],
			inputCalibration[1],
			inputCalibration[2],
			inputCalibration[3]);
	}

	cv::Mat originalK(3, 3, CV_64F, cv::Scalar(0));
	originalK.at<double>(0, 0) = inputCalibration[0];
	originalK.at<double>(1, 1) = inputCalibration[1];
	originalK.at<double>(2, 2) = 1;
	originalK.at<double>(0, 2) = inputCalibration[2];
	originalK.at<double>(1, 2) = inputCalibration[3];

	if (valid)
	{
		return new OpenCVUndistorter( originalK, distCoeffs, ImageSize( in_width, in_height ), wrap);
    //
		// K_ = cv::getOptimalNewCameraMatrix(originalK_, distCoeffs, cv::Size(in_width, in_height), (outputCalibration == -2) ? 1 : 0, cv::Size(out_width, out_height), nullptr, false);
    //
		// cv::initUndistortRectifyMap(originalK_, distCoeffs, cv::Mat(), K_,
		// 		cv::Size(out_width, out_height), CV_16SC2, map1, map2);
    //
		// originalK_.at<double>(0, 0) /= in_width;
		// originalK_.at<double>(0, 2) /= in_width;
		// originalK_.at<double>(1, 1) /= in_height;
		// originalK_.at<double>(1, 2) /= in_height;
	}

	// originalK_ = originalK_.t();
	// K_ = K_.t();

	return nullptr;
}

}
