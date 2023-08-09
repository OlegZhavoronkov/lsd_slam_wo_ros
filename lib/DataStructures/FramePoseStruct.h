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
#include <memory>
//#define USE_CONDITIONAL_DEBUG_BREAK
#include "util/globalFuncs.h"
#include "util/SophusUtil.h"
#include "GlobalMapping/g2oTypeSim3Sophus.h"


namespace lsd_slam
{
class Frame;

class FramePoseStruct {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<FramePoseStruct> SharedPtr;

	FramePoseStruct( Frame &frame );
	virtual ~FramePoseStruct();

	FramePoseStruct &operator=( const FramePoseStruct &other );

	Sim3 setThisToParent( const Sim3 &val );

	// set initially as tracking result (then it's a SE(3)),
	// and is changed only once, when the frame becomes a KF (->rescale).
//	Sim3 _thisToParent_raw;

 	Frame &frame;
	//int frameID;

	// whether this poseStruct is registered in the Graph. if true MEMORY WILL BE HANDLED BY GRAPH
	bool isRegisteredToGraph;

	// whether pose is optimized (true only for KF, after first applyPoseGraphOptResult())
	bool isOptimized;

	// true as soon as the vertex is added to the g2o graph.
	bool isInGraph;

	// graphVertex (if the frame has one, i.e. is a KF and has been added to the graph, otherwise 0).
	VertexSim3* graphVertex;

	void setPoseGraphOptResult(Sim3 camToWorld);
	void applyPoseGraphOptResult();

	Sim3 getCamToWorld(int recursionDepth = 0);
	void invalidateCache();
    /*inline*/ Sim3& setThisToParent_raw(const Sim3& newParentRaw);

    inline const Sim3& getThisToParent_raw()const
    {
        auto tr=_thisToParent_raw.translation().cast<float>();
        (void)tr;
        CONDITIONAL_BREAK(!((abs(tr[2]) > 2*abs(tr[1])) && (abs(tr[2]) > 2*abs(tr[0]))));
        return std::forward<const Sim3&>(_thisToParent_raw);
    }

private:
	int cacheValidFor;
	static int cacheValidCounter;

	// absolute position (camToWorld).
	// can change when optimization offset is merged.
	Sim3 camToWorld;

	// new, optimized absolute position. is added on mergeOptimization.
	Sim3 camToWorld_new;

	// whether camToWorld_new is newer than camToWorld
	bool hasUnmergedPose;

    	// set initially as tracking result (then it's a SE(3)),
	// and is changed only once, when the frame becomes a KF (->rescale).
	Sim3 _thisToParent_raw;

};

}
