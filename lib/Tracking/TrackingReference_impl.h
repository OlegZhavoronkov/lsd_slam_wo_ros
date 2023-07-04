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

#include "DataStructures/Frame.h"
#include "util/globalFuncs.h"

namespace lsd_slam
{
template< int __LEVELS >
_TrackingRef< __LEVELS >::_TrackingRef( const Frame::SharedPtr &frame )
	: frame( frame ),
		_accessMutex()
{
	wh_allocated = 0;
	for (int level = 0; level < __LEVELS; ++ level)
	{
		posData[level] = nullptr;
		gradData[level] = nullptr;
		colorAndVarData[level] = nullptr;
		pointPosInXYGrid[level] = nullptr;
		numData[level] = 0;
	}
}

template< int __LEVELS >
_TrackingRef<__LEVELS>::~_TrackingRef()
{
	std::lock_guard<std::mutex> lock(_accessMutex);

	for (int level = 0; level < __LEVELS; ++ level)
	{
		if(posData[level] != nullptr) delete[] posData[level];
		if(gradData[level] != nullptr) delete[] gradData[level];
		if(colorAndVarData[level] != nullptr) delete[] colorAndVarData[level];
		if(pointPosInXYGrid[level] != nullptr) delete[] pointPosInXYGrid[level];
		numData[level] = 0;
	}
	wh_allocated = 0;
}

template< int __LEVELS >
void _TrackingRef< __LEVELS >::clearAll()
{
	for (int level = 0; level < __LEVELS; ++level)
		numData[level] = 0;
}


template< int __LEVELS >
void _TrackingRef< __LEVELS >::makePointCloud(int level)
{
	CHECK( (bool)frame ) << "frame pointer is NULL when it shouldn't be.";
	std::lock_guard<std::mutex> lock(_accessMutex);

	if(numData[level] > 0) return;	// already exists.

	const int w = frame->width(level);
	const int h = frame->height(level);

	const float fxInvLevel = frame->fxi(level);
	const float fyInvLevel = frame->fyi(level);
	const float cxInvLevel = frame->cxi(level);
	const float cyInvLevel = frame->cyi(level);

	const float* pyrIdepthSource = frame->idepth(level);
	const float* pyrIdepthVarSource = frame->idepthVar(level);
	const float* pyrColorSource = frame->image(level);
	const Eigen::Vector4f* pyrGradSource = frame->gradients(level);

	if(posData[level] == nullptr)          posData[level] = new Eigen::Vector3f[w*h];
	if(pointPosInXYGrid[level] == nullptr) pointPosInXYGrid[level] = new int[w*h];
	if(gradData[level] == nullptr)         gradData[level] = new Eigen::Vector2f[w*h];
	if(colorAndVarData[level] == nullptr)  colorAndVarData[level] = new Eigen::Vector2f[w*h];

	Eigen::Vector3f* posDataPT = posData[level];
	int* idxPT = pointPosInXYGrid[level];
	Eigen::Vector2f* gradDataPT = gradData[level];
	Eigen::Vector2f* colorAndVarDataPT = colorAndVarData[level];

	for(int x=1; x<w-1; x++) {
		for(int y=1; y<h-1; y++) {
			int idx = x + y*w;

			if(pyrIdepthVarSource[idx] <= 0 || pyrIdepthSource[idx] == 0) continue;

			*posDataPT = (1.0f / pyrIdepthSource[idx]) * Eigen::Vector3f(fxInvLevel*x+cxInvLevel,fyInvLevel*y+cyInvLevel,1);
			*gradDataPT = pyrGradSource[idx].head<2>();
			*colorAndVarDataPT = Eigen::Vector2f(pyrColorSource[idx], pyrIdepthVarSource[idx]);
			*idxPT = idx;

			posDataPT++;
			gradDataPT++;
			colorAndVarDataPT++;
			idxPT++;
		}
	}

	numData[level] = posDataPT - posData[level];
	LOG(INFO) << "frame " << frameID() << " has " << numData[level] << " tracked points at level " << level;
}


}
