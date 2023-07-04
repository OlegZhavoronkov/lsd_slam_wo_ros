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
#include "util/settings.h"
#include "util/EigenCoreInclude.h"
#include "boost/thread/mutex.hpp"
#include <boost/thread/shared_mutex.hpp>

#include "DataStructures/Frame.h"

namespace lsd_slam
{


class DepthMapPixelHypothesis;
class KeyFrameGraph;

/**
 * Point cloud used to track frame poses.
 *
 * This stores a point cloud generated from known frames. It is used to
 * track a new frame by finding a projection of the point cloud which makes it
 * look as much like the new frame as possible.
 *
 * It is intended to use more than one old frame as source for the point cloud.
 * Also other data like Kinect depth data could be imported.
 *
 * ATTENTION: as the level zero point cloud is not used for tracking, it is not
 * fully calculated. Only the weights are valid on this level!
 */

 // TODO.  Oh my gawd.
template< int __LEVELS >
class _TrackingRef
{
public:

	typedef std::shared_ptr<_TrackingRef> SharedPtr;

	_TrackingRef() = delete;
	_TrackingRef( const _TrackingRef & ) = delete;

	_TrackingRef( const Frame::SharedPtr &frame );

	~_TrackingRef();

	int frameID()    { return ((bool)frame) ? frame->id() : -1; }

	Frame::SharedPtr frame;

	void makePointCloud(int level);
	void clearAll();

	Eigen::Vector3f* posData[__LEVELS];	// (x,y,z)
	Eigen::Vector2f* gradData[__LEVELS];	// (dx, dy)
	Eigen::Vector2f* colorAndVarData[__LEVELS];	// (I, Var)
	int* pointPosInXYGrid[__LEVELS];	// x + y*width
	int numData[__LEVELS];

private:
	int wh_allocated;
	std::mutex _accessMutex;
};

typedef _TrackingRef<PYRAMID_LEVELS> TrackingReference;

}

#include "TrackingReference_impl.h"
