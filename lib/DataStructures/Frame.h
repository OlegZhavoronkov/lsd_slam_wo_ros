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
#include <mutex>
#include "util/SophusUtil.h"
#include "util/settings.h"
#include <shared_mutex>
#include "DataStructures/FramePoseStruct.h"
#include "DataStructures/FrameMemory.h"
#include "unordered_set"
#include "util/settings.h"
#include "util/Configuration.h"

#include "FrameData.h"

namespace lsd_slam
{

using libvideoio::Camera;
using libvideoio::ImageSize;

class KeyFrame;
class DepthMapPixelHypothesis;
class DepthMap;

template< int __LEVELS > class _TrackingRef;
typedef _TrackingRef<PYRAMID_LEVELS> TrackingReference;


/**
 */

class Frame
{
private:
	// Again, having wierd FrameData alignment issues where it's of different
	// lengths in different subunits unless it's at the top...
	FrameData<PYRAMID_LEVELS> data;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	friend class FrameMemory;

	typedef std::shared_ptr<Frame> SharedPtr;

	// Explicitly delete default and copy constructors
	Frame() = delete;
	Frame( const Frame & ) = delete;

	Frame(int id, const Camera &cam, const ImageSize &sz, double timestamp, const unsigned char* image );
	Frame(int id, const Camera &cam, const ImageSize &sz, double timestamp, const float* image );

	~Frame();

	/** Sets or updates idepth and idepthVar on level zero. Invalidates higher levels. */
	void setDepth(const std::shared_ptr<DepthMap> &depthMap ); //PixelHypothesis* newDepth);

	/** Calculates mean information for statistical purposes. */
	void calculateMeanInformation();

	/** Sets ground truth depth (real, not inverse!) from a float array on level zero. Invalidates higher levels. */
	void setDepthFromGroundTruth(const float* depth, float cov_scale = 1.0f);

	/** Prepares this frame for stereo comparisons with the other frame
	//(computes some intermediate values that will be needed) */
	void prepareForStereoWith( const Frame::SharedPtr &other, Sim3 thisToOther, const int level);



	// Accessors
	/** Returns the unique frame id. */

	inline int id() const { return _id; }

	#define DATA_LEVEL_READER( _rtype, _name ) \
		inline _rtype _name( int level = 0 ) const \
			{ return data._name[level]; }

	#define DATA_LEVEL_CAMERA_READER( _rtype, _name ) \
		inline _rtype _name( int level = 0 ) const \
			{ return data.camera[level]._name; }

	inline const ImageSize &imgSize( int i=0 ) const 	{ return data.imgSize[i]; }
	inline int width( int i=0 ) const	 								{ return data.imgSize[i].width; }
	inline int height( int i=0 ) const 								{ return data.imgSize[i].height; }
	inline int area( int i=0 ) const 	 								{ return data.imgSize[i].area(); }


	DATA_LEVEL_READER( const Camera &, camera )

	DATA_LEVEL_CAMERA_READER( const Eigen::Matrix3f&, K )
	DATA_LEVEL_CAMERA_READER( const Eigen::Matrix3f&, Kinv )
	DATA_LEVEL_CAMERA_READER( float, fx )
	DATA_LEVEL_CAMERA_READER( float, fy )
	DATA_LEVEL_CAMERA_READER( float, cx )
	DATA_LEVEL_CAMERA_READER( float, cy )
	DATA_LEVEL_CAMERA_READER( float, fxi )
	DATA_LEVEL_CAMERA_READER( float, fyi )
	DATA_LEVEL_CAMERA_READER( float, cxi )
	DATA_LEVEL_CAMERA_READER( float, cyi )

	/** Returns the frame's recording timestamp. */
	inline double timestamp() const { return _timestamp; }

	inline float* image(int level = 0);
	inline const Eigen::Vector4f* gradients(int level = 0);
	inline const float* maxGradients(int level = 0);
	inline bool hasIDepthBeenSet() const;
	inline const float* idepth(int level = 0);
	inline const float* idepthVar(int level = 0);
	inline const unsigned char* validity_reAct();
	inline const float* idepth_reAct();
	inline const float* idepthVar_reAct();

	inline bool* refPixelWasGood();
	inline const bool* refPixelWasGoodNoCreate();
	inline void  clear_refPixelWasGood();

	/** Flags for use with require() and requirePyramid(). See the Frame class
	  * documentation for their exact meaning. */
	enum DataFlags
	{
		IMAGE			= 1<<0,
		GRADIENTS		= 1<<1,
		MAX_GRADIENTS	= 1<<2,
		IDEPTH			= 1<<3,
		IDEPTH_VAR		= 1<<4,
		REF_ID			= 1<<5,

		ALL = IMAGE | GRADIENTS | MAX_GRADIENTS | IDEPTH | IDEPTH_VAR | REF_ID
	};


  // For SLAM-like features, KeyFrames can own their own TrackingReference
	// this is copied into the keyframe when the keyframe is finalized
	// This used for loop closure and re-localization
	// void setPermaRef( const std::unique_ptr<TrackingReference> &reference);
	// void takeReActivationData(DepthMapPixelHypothesis* depthMap);


	// shared_lock this as long as any minimizable arrays are being used.
	// the minimizer will only minimize frames after getting
	// an exclusive lock on this.
	inline std::shared_lock<std::shared_timed_mutex> getActiveLock()
	{
		return FrameMemory::getInstance().activateFrame(this);
	}

	std::mutex frameMutex;


	/*
	 * ==================================================================================
	 * Here are ALL central pose and scale informations.
	 * generally, everything is stored relative to the frame
	 */
	FramePoseStruct::SharedPtr pose;
	Sim3 getCamToWorld()  { return pose->getCamToWorld(); }


	// parent, the frame originally tracked on. never changes.
	void setTrackingParent( const std::shared_ptr<KeyFrame> &newParent  ) ;

	bool      hasTrackingParent() const     															{ return (bool)_trackingParent; }
	const std::shared_ptr<KeyFrame> &trackingParent() const       				{ return _trackingParent; }

	bool isTrackingParent( const std::shared_ptr<Frame> &other ) const;
	bool isTrackingParent( const std::shared_ptr<KeyFrame> &other ) const;
	bool isTrackingParent( int id ) const;


	Sim3 lastConstraintTrackedCamToWorld;

	// flag set when depth is updated.
	//bool depthHasBeenUpdatedFlag;


	// Tracking Reference for quick test. Always available, never taken out of memory.
	// this is used for re-localization and re-Keyframe positioning.
	// std::mutex permaRef_mutex;
	// Eigen::Vector3f* permaRef_posData;	// (x,y,z)
	// Eigen::Vector2f* permaRef_colorAndVarData;	// (I, Var)
	// int permaRefNumPts;



	// A bunch of state which is created by prepareForStereoWith()
	int referenceID;
	int referenceLevel;
	float _distSquared;
	Eigen::Matrix3f _K_otherToThis_R;
	Eigen::Vector3f _K_otherToThis_t;
	Eigen::Vector3f _otherToThis_t;
	Eigen::Vector3f _K_thisToOther_t;
	Eigen::Matrix3f _thisToOther_R;
	Eigen::Vector3f _otherToThis_R_row0;
	Eigen::Vector3f _otherToThis_R_row1;
	Eigen::Vector3f _otherToThis_R_row2;
	Eigen::Vector3f _thisToOther_t;



	// statistics
	float initialTrackedResidual;
	float meanIdepth;
	int numPoints;
	int idxInKeyframes;
	float edgeErrorSum, edgesNum;
	int numMappablePixels;
	float meanInformation;

private:

	std::shared_ptr<KeyFrame> _trackingParent;

	int _id;
	double _timestamp;

	void require(int dataFlags, int level = 0);
	void release(int dataFlags, bool pyramidsOnly, bool invalidateOnly);

//	void initialize(double timestamp);
	void setDepth_Allocate();

	void buildImage(int level);
	void releaseImage(int level);

	void buildGradients(int level);
	void releaseGradients(int level);

	void buildMaxGradients(int level);
	void releaseMaxGradients(int level);

	void buildIDepthAndIDepthVar(int level);
	void releaseIDepth(int level);
	void releaseIDepthVar(int level);

	// used internally. locked while something is being built, such that no
	// two threads build anything simultaneously. not locked on require() if nothing is changed.
	std::mutex buildMutex;

	std::shared_timed_mutex activeMutex;
	bool isActive;

	/** Releases everything which can be recalculated, but keeps the minimal
	  * representation in memory. Use release(Frame::ALL, false) to store on disk instead.
	  * ONLY CALL THIS, if an exclusive lock on activeMutex is owned! */
	bool minimizeInMemory();

};


inline float* Frame::image(int level)
{
	if (! data.imageValid[level])
		require(IMAGE, level);
	return data.image[level];
}

inline const Eigen::Vector4f* Frame::gradients(int level)
{
	if (! data.gradientsValid[level])
		require(GRADIENTS, level);
	return data.gradients[level];
}

inline const float* Frame::maxGradients(int level)
{
	if (! data.maxGradientsValid[level])
		require(MAX_GRADIENTS, level);
	return data.maxGradients[level];
}

inline bool Frame::hasIDepthBeenSet() const
{
	return data.hasIDepthBeenSet;
}

inline const float* Frame::idepth(int level)
{
	if (! hasIDepthBeenSet())
	{
		LOG(WARNING) << "Frame " <<id() << "; idepth(): idepth has not been set yet!";
		return nullptr;
	}
	if (! data.idepthValid[level])
		require(IDEPTH, level);
	return data.idepth[level];
}

inline const unsigned char* Frame::validity_reAct()
{
	if( !data.reActivationDataValid)
		return 0;
	return data.validity_reAct;
}

inline const float* Frame::idepth_reAct()
{
	if( !data.reActivationDataValid)
		return 0;
	return data.idepth_reAct;
}

inline const float* Frame::idepthVar_reAct()
{
	if( !data.reActivationDataValid)
		return 0;
	return data.idepthVar_reAct;
}

inline const float* Frame::idepthVar(int level)
{
	if (! hasIDepthBeenSet())
	{
		LOG(WARNING) << "Frame " << id() << "; idepthVar(): idepth has not been set yet!";
		return nullptr;
	}
	if (! data.idepthVarValid[level]) require(IDEPTH_VAR, level);
	return data.idepthVar[level];
}


inline bool* Frame::refPixelWasGood()
{
	if( data.refPixelWasGood == 0)
	{
		std::unique_lock<std::mutex> lock2(buildMutex);

		if(data.refPixelWasGood == 0)
		{
			const int width = data.imgSize[SE3TRACKING_MIN_LEVEL].width;
			const int height = data.imgSize[SE3TRACKING_MIN_LEVEL].height;
			data.refPixelWasGood = (bool*)FrameMemory::getInstance().getBuffer(sizeof(bool) * width * height);
            /*on gcc 10.3 this tends to mess during tracking and abnormal tracking finished*/
			memset(data.refPixelWasGood, (int)true/*0xFFFFFFFF*/, sizeof(bool) * (width * height));
		}
	}
	return data.refPixelWasGood;
}


inline const bool* Frame::refPixelWasGoodNoCreate()
{
	return data.refPixelWasGood;
}

inline void Frame::clear_refPixelWasGood()
{
	FrameMemory::getInstance().returnBuffer(reinterpret_cast<float*>(data.refPixelWasGood));
	data.refPixelWasGood=0;
}


}
