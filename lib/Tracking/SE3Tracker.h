/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <opencv2/core/core.hpp>
#include "util/SE3TrackerSignals.h"
#include "util/DenseDepthTrackerSettings.h"
#include "util/EigenCoreInclude.h"
#include "util/SophusUtil.h"
#include "util/Configuration.h"
#include "Tracking/LGSX.h"


namespace lsd_slam
{

template< int __LEVELS > class _TrackingRef;
typedef _TrackingRef<PYRAMID_LEVELS> TrackingReference;

class Frame;
class KeyFrame;



class SE3Tracker:public util::SE3TrackerSignals
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	DenseDepthTrackerSettings<PYRAMID_LEVELS> settings;

	SE3Tracker(const SE3Tracker&) = delete;
	SE3Tracker& operator=(const SE3Tracker&) = delete;

	SE3Tracker( const ImageSize &sz );
	~SE3Tracker();

	// Trackes _frame_ onto _keyframe_.  Modifies _frame_
	SE3 trackFrame(
			const std::shared_ptr<KeyFrame> &keyframe,
			const std::shared_ptr<Frame> &frame,
			const SE3& frameToReference_initialEstimate);

	// This version does not recurse through levels of the pyramid,
	// It only checks QUICK_KF_CHECK_LVL
	SE3 trackFrameOnPermaref(
			const std::shared_ptr<KeyFrame> &reference,
			const std::shared_ptr<Frame> &frame,
			SE3 referenceToFrame );

	// Calculates the percentage overlap between the two keyframes
	float checkPermaRefOverlap(
				const std::shared_ptr<KeyFrame> &reference,
				SE3 referenceToFrame );


	float pointUsage;
	float lastGoodCount() const { return _lastGoodCount; }
	float lastMeanRes;
	float lastBadCount() const { return _lastBadCount; }
	float lastResidual;

	float _pctGoodPerGoodBad;
	float _pctGoodPerTotal;

	float affineEstimation_a;
	float affineEstimation_b;


	bool _diverged;
	bool _trackingWasGood;
private:

	float _lastGoodCount;
	float _lastBadCount;

	const ImageSize &_imgSize;

	float* buf_warped_residual;
	float* buf_warped_dx;
	float* buf_warped_dy;
	float* buf_warped_x;
	float* buf_warped_y;
	float* buf_warped_z;

	float* buf_d;
	float* buf_idepthVar;
	float* buf_weight_p;

	int buf_warped_size;

	//SE3TrackerDebugImages _debugImages;


	void calculateWarpUpdate(LGS6D &ls);

	float calcWeightsAndResidual(	const Sophus::SE3f& referenceToFrame);

	float calcResidualAndBuffers(
			const Eigen::Vector3f* refPoint,
			const Eigen::Vector2f* refColVar,
			int* idxBuf,
			int refNum,
			const std::shared_ptr<Frame> &frame,
			const Sophus::SE3f& referenceToFrame,
			int level,
			bool plotResidual = false);


#if defined(ENABLE_SSE)
	void calculateWarpUpdateSSE(LGS6 &ls);
	float calcWeightsAndResidualSSE(const Sophus::SE3f& referenceToFrame);
	float calcResidualAndBuffersSSE(
			const Eigen::Vector3f* refPoint,
			const Eigen::Vector2f* refColVar,
			int* idxBuf,
			int refNum,
			const std::shared_ptr<Frame> &frame,
			const Sophus::SE3f& referenceToFrame,
			int level,
			bool plotResidual = false);
#endif

#if defined(ENABLE_NEON)
	void calculateWarpUpdateNEON(LGS6 &ls);
	float calcWeightsAndResidualNEON(const Sophus::SE3f& referenceToFrame);
	float calcResidualAndBuffersNEON(
			const Eigen::Vector3f* refPoint,
			const Eigen::Vector2f* refColVar,
			int* idxBuf,
			int refNum,
			const std::shared_ptr<Frame> &frame,
			const Sophus::SE3f& referenceToFrame,
			int level,
			bool plotResidual = false);
#endif
    using CalcResidualInitStruct=std::tuple<    cv::Mat*  ,  /*pDebugImageOldImageSource*/
                                                cv::Mat*  ,  /*pDebugImageOldImageWarped*/
                                                cv::Mat*    /*pdebugImageResiduals,*/       >;
	CalcResidualInitStruct calcResidualAndBuffers_debugStart(std::mutex& mtx);
	void calcResidualAndBuffers_debugFinish(int w,int loop,int buf_warped_size,int goodCount ,int badCount,float ratio);


	// used for image saving
	int iterationNumber;


	float affineEstimation_a_lastIt;
	float affineEstimation_b_lastIt;
public:
    OnCalcResidualStartedSignal         _OnCalcResidualStartedSignal        ;
    OnCalcResidualFinishedSignal        _OnCalcResidualFinishedSignal       ;
    OnCalcResidualErrorCalculatedSignal _OnCalcResidualErrorCalculatedSignal;

    OnCalcResidualAndBuffersDebugStart  _OnCalcResidualAndBuffersDebugStart;
    OnCalcResidualAndBuffersDebugFinish  _OnCalcResidualAndBuffersDebugFinish;
private:
    void DebugPlotTrackingAndResidualInfo(const int width,const Eigen::Matrix3f& KLvl,
                                            cv::Mat* pDebugImageOldImageSource,
                                            cv::Mat* pDebugImageOldImageWarped,
                                            cv::Mat* pdebugImageResiduals,
                                            const int wThisLvl,
                                            const int hThisLvl,
                                            const Eigen::Vector3f* pRefPoint,
		                                    const Eigen::Vector2f* refColVar,
                                            int* idxBuf,
                                            const Eigen::Vector3f* refPoint_max,
                                            const Eigen::Matrix3f& rotMat,
                                            const Eigen::Vector3f& transVec,
                                            bool* isGoodOutBuffer,
                                            const Eigen::Vector4f* frame_gradients);
    std::mutex _CalcResidualAndBuffersDebugSignalMtx;
};


}
