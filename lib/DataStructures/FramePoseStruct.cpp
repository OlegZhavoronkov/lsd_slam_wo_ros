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


#include <DataStructures/FramePoseStruct.h>
#include "DataStructures/Frame.h"
#include "DataStructures/KeyFrame.h"

namespace lsd_slam
{

int FramePoseStruct::cacheValidCounter = 0;

//int privateFramePoseStructAllocCount = 0;

FramePoseStruct::FramePoseStruct( Frame &f )
	:frame( f ),
	 graphVertex( nullptr )
{
	cacheValidFor = -1;
	isOptimized = false;
	_thisToParent_raw = camToWorld = camToWorld_new = Sim3();
	isRegisteredToGraph = false;
	hasUnmergedPose = false;
	isInGraph = false;

	// privateFramePoseStructAllocCount++;
	// LOG_IF(INFO, Conf().print.memoryDebugInfo) << "ALLOCATED pose for frame " << frame.id() << ", " << privateFramePoseStructAllocCount << " poses still allocated";
}

FramePoseStruct::~FramePoseStruct()
{
	// privateFramePoseStructAllocCount--;
	// LOG_IF(INFO, Conf().print.memoryDebugInfo) << "DELETED pose for frame " << frame.id() << ", " << privateFramePoseStructAllocCount << " poses still allocated";
}

Sim3 FramePoseStruct::setThisToParent( const Sim3 &val )
{
	_thisToParent_raw = val;
	invalidateCache();
	return _thisToParent_raw;
}

FramePoseStruct &FramePoseStruct::operator=( const FramePoseStruct &other )
{
	_thisToParent_raw = other._thisToParent_raw;
	invalidateCache();
	return *this;
}


void FramePoseStruct::setPoseGraphOptResult(Sim3 camToWorld)
{
	if(!isInGraph)
		return;

	camToWorld_new = camToWorld;
	hasUnmergedPose = true;
}

void FramePoseStruct::applyPoseGraphOptResult()
{
	if(!hasUnmergedPose)
		return;

	camToWorld = camToWorld_new;
	isOptimized = true;
	hasUnmergedPose = false;
	cacheValidCounter++;
}

void FramePoseStruct::invalidateCache()
{
	cacheValidFor = -1;
}

Sim3 FramePoseStruct::getCamToWorld(int recursionDepth)
{
	// prevent stack overflow
	assert(recursionDepth < 5000);

	// if the node is in the graph, it's absolute pose is only changed by optimization.
	if(isOptimized) return camToWorld;

	// return chached pose, if still valid.
	if(cacheValidFor == cacheValidCounter)
		return camToWorld;

	// return identity if there is no parent (very first frame)
	if( frame.hasTrackingParent() ) {
			LOG(DEBUG) << "Frame " << frame.id() << ": Calculating pose from tracked parent...";
			// abs. pose is computed from the parent's abs. pose, and cached.
			cacheValidFor = cacheValidCounter;
			return camToWorld = frame.trackingParent()->frame()->pose->getCamToWorld(recursionDepth+1) * _thisToParent_raw;
	} else {
		LOG(DEBUG) << "Frame " << frame.id() << ": No parent, returning identity pose";

		return camToWorld = Sim3();
	}
}

}
