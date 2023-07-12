/**
 * This file is part of LSD-SLAM.
 *
 * Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical
 * University of Munich) For more information see
 * <http://vision.in.tum.de/lsdslam>
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
#include <boost/signals2.hpp>
#include "opencv2/core/core.hpp"
#include "util/Configuration.h"
#include "util/EigenCoreInclude.h"
#include "util/IndexThreadReduce.h"
#include "util/MovingAverage.h"
#include "util/SophusUtil.h"
#include "util/Timer.h"
#include "util/settings.h"

#include "DataStructures/Frame.h"
#include "DataStructures/ImageSet.h"
#include "DepthMapPixelHypothesis.h"
#include "LineStereoResult.h"
namespace lsd_slam {

class DepthMapPixelHypothesis;
class KeyFrameGraph;

class KeyFrame;

/**
 * Keeps a detailed depth map (consisting of DepthMapPixelHypothesis) and does
 * stereo comparisons and regularization to update it.
 */
class DepthMap {
public:

  typedef std::shared_ptr<DepthMap> SharedPtr;

  // Delete default constructors
  DepthMap(const DepthMap &) = delete;
  DepthMap &operator=(const DepthMap &) = delete;

	DepthMap( const std::shared_ptr<Frame> &parent );

	// //== Propagation constructor for subsequent keyframes
	// DepthMap( const DepthMap::SharedPtr &other, const Frame::SharedPtr &frame );

	~DepthMap();

	/** Resets everything. */
	void reset();

	// /**
	//  * does obervation and regularization only.
	//  **/
	// void updateKeyframe(std::deque< Frame::SharedPtr > referenceFrames);
	//
	// /**
	//  * does propagation and whole-filling-regularization (no observation, for that need to call updateKeyframe()!)
	//  **/
	// void createKeyFrame( const Frame::SharedPtr &new_keyframe );

	/**
	 * does one fill holes iteration
	 */
	// void finalizeKeyFrame();

	// void invalidate();
	// inline bool isValid() {return (bool)activeKeyFrame;};

	//int debugPlotDepthMap();
	//const DepthMapDebugImages &debugImages() const { return *_debugImages; }

	// This is the only debug plot which is triggered externally..
	void plotDepthMap( const char *buf1, const char *buf2 );


	//== Initializers, required only when propagating from a previous keyframe
	void initializeFromFrame();
	void initializeFromGTDepth();
  void initializeFromStereo();
	void initializeRandomly();
	// void initializefromStereo( const std::shared_ptr<ImageSet> &set);

	void propagateFrom( const DepthMap::SharedPtr &new_keyframe, float &rescaleFactor );

	void finalize();

	// void activateExistingKF(const Frame::SharedPtr &kf);

  void activateExistingKF(const Frame::SharedPtr &kf);

	//==
	bool updateDepthFrom( const Frame::SharedPtr &frame );

  struct PerformanceData {
    PerformanceData(void) { ; }

    MsRateAverage update, create, finalize, observe, regularize, propagate,
        fillHoles, setDepth;
  };

  PerformanceData perf() const { return _perf; }
  void logPerformanceData();

	// Convenience accessors
	std::shared_ptr<Frame> &frame() { return _frame; }

	//
	DepthMapPixelHypothesis *hypothesisAt( const int x, const int y )
		{ return _pCurrentDepthMap + x + y*Conf().slamImageSize.width; }



private:
	IndexThreadReduce _threadReducer;

 	PerformanceData _perf;
	//std::unique_ptr< DepthMapDebugImages > _debugImages;

	// ============= parameter copies for convenience ===========================
	// these are just copies of the pointers given to this function, for convenience.
	// these are NOT managed by this object!
	std::shared_ptr<Frame> _frame;

	const float* activeKeyFrameImageData()    { return frame()->image(0); }
	bool activeKeyFrameIsReactivated;

	// Frame::SharedPtr oldest_referenceFrame;
	// Frame::SharedPtr newest_referenceFrame;
	// std::vector< Frame::SharedPtr > referenceFrameByID;
	// int referenceFrameByID_offset;

	// ============= internally used buffers for intermediate calculations etc. =============
	// for internal depth tracking, their memory is managed (created & deleted) by this object.
	DepthMapPixelHypothesis* otherDepthMap;
	DepthMapPixelHypothesis* _pCurrentDepthMap;
	int* validityIntegralBuffer;


    struct LineStereoDebugPlotStruct
    {
        cv::Point2f nearestEnd;
        cv::Point2f farEnd;
        cv::Scalar color;
        int IsValid=0;
    };
	// ============ internal functions ==================================================
	// does the line-stereo seeking.
	// takes a lot of parameters, because they all have been pre-computed before.
	inline float doLineStereo(
			const float u, const float v, const float epxn, const float epyn,
			const float min_idepth, const float prior_idepth, float max_idepth,
			const Frame* const referenceFrame, const float* referenceFrameImage,
			float &result_idepth, float &result_var, float &result_eplLength,
			RunningStats* const stats,LineStereoResult& res,LineStereoDebugPlotStruct *pPlotStruct);

	// Reset currentDepthMap by re-projecting is from activeKeyFrame to new_keyframe
	void propagateDepthFrom(const DepthMap::SharedPtr &new_keyframe, float &rescaleFactor );


	// This is a local state variable used to share data between the observeDepth* functions.  Sucks, I know
	Frame::SharedPtr _observeFrame;
	void observeDepth( const Frame::SharedPtr &updateFrame );
    using SetHypothesisHandlingFunctor = void(DepthMap::*)( int /*x*/, int /*y*/,const cv::Vec3b& /*color*/,LineStereoResult,cv::Mat* pDebugMat);
    using SetLineStereoDebugPlotFunctor = void(DepthMap::*)( const cv::Point2f& /*nearest*/,const cv::Point2f& closest,const cv::Scalar& /*color*/,cv::Mat* pDebugMat);
	void observeDepthRow(const SetHypothesisHandlingFunctor pFunctor,void* pToDebugMat,const SetLineStereoDebugPlotFunctor pLineFunctor,void* pToLineMat,int yMin, int yMax, RunningStats* stats);
	LineStereoResult observeDepthCreate(const int &x, const int &y, const int &idx, RunningStats* const &stats,LineStereoDebugPlotStruct* pPlotDebugLine );
	LineStereoResult observeDepthUpdate(const int &x, const int &y, const int &idx, const float* keyFrameMaxGradBuf, RunningStats* const &stats , LineStereoDebugPlotStruct* pPlotDebugLine);
	LineStereoResult makeAndCheckEPL(const int x, const int y, const Frame* const ref, float* pepx, float* pepy, RunningStats* const stats);


	void regularizeDepthMap(bool removeOcclusion, int validityTH);
	template<bool removeOcclusions> void regularizeDepthMapRow(int validityTH, int yMin, int yMax, RunningStats* stats);


	void buildRegIntegralBuffer();
	void buildRegIntegralBufferRow1(int yMin, int yMax, RunningStats* stats);
	void regularizeDepthMapFillHoles();
	void regularizeDepthMapFillHolesRow(int yMin, int yMax, RunningStats* stats);


	void resetCounters();

	//float clocksPropagate, clocksPropagateKF, clocksObserve, msObserve, clocksReg1, clocksReg2, msReg1, msReg2, clocksFinalize;
public :
    using UpdateKeyFrameSignal = boost::signals2::signal<void(const cv::Mat& /*debugDepthRes*/,const cv::Mat& /*lineStereoDebug*/)>;
    using DisplayUpdateKeyFrameSignal = boost::signals2::signal<void()>;
    using PlotDepthSignal               = boost::signals2::signal<void(const DepthMapPixelHypothesis*,int,const char*,const char*)>;
    //using DisplayNewKeyFrameSignal = boost::signals2::signal<void()>;
private:
    UpdateKeyFrameSignal _updateKeyFrameSignal;
    DisplayUpdateKeyFrameSignal _displayUpdateKeyFrameSignal;
    PlotDepthSignal _plotDepthSignal;
    //DisplayNewKeyFrameSignal    _displayNewKeyFrameSignal;

    template<typename SignalType,typename SignalInvokable> std::enable_if_t<
            std::is_void_v<
                std::void_t<
                        decltype(
                                std::declval<SignalType>()
                                    .connect(std::declval<SignalInvokable>())
                                )
                            > >,
             SignalType&> ConnectSignal(SignalType DepthMap::* pSignalMember,SignalInvokable invokable)
    {
        this->*pSignalMember.connect(invokable);
        //_updateKeyFrameSignal.connect(invokable);
        return std::forward<SignalType&>(this->*pSignalMember);
    }

public:


    template<typename SignalInvokable> UpdateKeyFrameSignal& ConnectUpdateKeyFrameSignal(SignalInvokable invokable)
    {
        return ConnectSignal(&DepthMap::_updateKeyFrameSignal,invokable);
    }
    template<typename SignalInvokable> DisplayUpdateKeyFrameSignal& ConnectDisplayUpdateKeyFrameSignal(SignalInvokable invokable)
    {
        return ConnectSignal(&DepthMap::_displayUpdateKeyFrameSignal,invokable);
    }
    template<typename SignalInvokable> PlotDepthSignal&  ConnectPlotDepthSignal(SignalInvokable invokable)
    {
        return ConnectSignal(&DepthMap::_plotDepthSignal,invokable);
    }
private:
    void SetHypotesisDebugData( int x, int y,const cv::Vec3b& /*color*/,LineStereoResult res,cv::Mat* pDebugHypotesisMat);
    void SetLineStereoDebugData( const cv::Point2f& nearestPoint, const cv::Point2f& farestPoint,const cv::Scalar& color,cv::Mat* pDebugMat);
};

} // namespace lsd_slam
