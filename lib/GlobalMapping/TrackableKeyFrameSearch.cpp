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

#include "GlobalMapping/TrackableKeyFrameSearch.h"


#include "GlobalMapping/KeyFrameGraph.h"
#include "DataStructures/Frame.h"
#include "Tracking/SE3Tracker.h"

namespace lsd_slam
{


TrackableKeyFrameSearch::TrackableKeyFrameSearch( const std::shared_ptr<KeyFrameGraph> &graph )
: graph(graph),
	tracker( new SE3Tracker( Conf().slamImageSize ) )
{

}

TrackableKeyFrameSearch::~TrackableKeyFrameSearch()
{
	;
}




std::vector<TrackableKFStruct> TrackableKeyFrameSearch::findEuclideanOverlapFrames(const KeyFrame::SharedPtr &keyframe, float distanceTH, float angleTH, bool checkBothScales)
{
	float fowX = 2 * atanf(float(Conf().slamImageSize.width) * keyframe->frame()->fxi() / 2.0f );
	float fowY = 2 * atanf(float(Conf().slamImageSize.height) * keyframe->frame()->fyi() / 2.0f );

	LOGF_IF(INFO, Conf().print.relocalizationInfo,
					"Relocalization Values: fowX %f, fowY %f\n", fowX, fowY);

	// basically the maximal angle-difference in viewing direction is angleTH*(average FoV).
	// e.g. if the FoV is 130°, then it is angleTH*130°.
	float cosAngleTH = cosf(angleTH*0.5f*(fowX + fowY));


	Eigen::Vector3d pos = keyframe->frame()->getCamToWorld().translation();
	Eigen::Vector3d viewingDir = keyframe->frame()->getCamToWorld().rotationMatrix().rightCols<1>();

	std::vector<TrackableKFStruct> potentialReferenceFrames;

	float distFacReciprocal = 1;
	if(checkBothScales)
		distFacReciprocal = keyframe->frame()->meanIdepth / keyframe->frame()->getCamToWorld().scale();

	// for each frame, calculate the rough score, consisting of pose, scale and angle overlap.
	graph->keyframesAllMutex.lock_shared();
	for(unsigned int i=0;i<graph->keyframesAll.size();i++)
	{
		Eigen::Vector3d otherPos = graph->keyframesAll[i]->frame()->getCamToWorld().translation();

		// get distance between the frames, scaled to fit the potential reference frame.
		float distFac = graph->keyframesAll[i]->frame()->meanIdepth / graph->keyframesAll[i]->frame()->getCamToWorld().scale();
		if(checkBothScales && distFacReciprocal < distFac) distFac = distFacReciprocal;
		Eigen::Vector3d dist = (pos - otherPos) * distFac;
		float dNorm2 = dist.dot(dist);
		if(dNorm2 > distanceTH) continue;

		Eigen::Vector3d otherViewingDir = graph->keyframesAll[i]->frame()->getCamToWorld().rotationMatrix().rightCols<1>();
		float dirDotProd = otherViewingDir.dot(viewingDir);
		if(dirDotProd < cosAngleTH) continue;

		potentialReferenceFrames.push_back(TrackableKFStruct());
		potentialReferenceFrames.back().keyframe = graph->keyframesAll[i];
		potentialReferenceFrames.back().refToFrame = se3FromSim3(graph->keyframesAll[i]->frame()->getCamToWorld().inverse() * keyframe->frame()->getCamToWorld()).inverse();
		potentialReferenceFrames.back().dist = dNorm2;
		potentialReferenceFrames.back().angle = dirDotProd;
	}
	graph->keyframesAllMutex.unlock_shared();

	return potentialReferenceFrames;
}




KeyFrame::SharedPtr TrackableKeyFrameSearch::findRePositionCandidate( const KeyFrame::SharedPtr &keyframe, float maxScore)
{
	std::vector<TrackableKFStruct> potentialReferenceFrames = findEuclideanOverlapFrames(keyframe, maxScore / (KFDistWeight*KFDistWeight), 0.75);

	float bestScore = maxScore;
	float bestDist, bestUsage;
	float bestPoseDiscrepancy = 0;
	KeyFrame::SharedPtr bestFrame(nullptr);
	SE3 bestRefToFrame = SE3();
	SE3 bestRefToFrame_tracked = SE3();

	int checkedSecondary = 0;
	for(unsigned int i=0;i<potentialReferenceFrames.size();i++)
	{
		if(keyframe->frame()->isTrackingParent( potentialReferenceFrames[i].keyframe->frame() ) )
			continue;

		if(potentialReferenceFrames[i].keyframe->frame()->idxInKeyframes < INITIALIZATION_PHASE_COUNT)
			continue;

		{
			Timer time;
			tracker->checkPermaRefOverlap(potentialReferenceFrames[i].keyframe, potentialReferenceFrames[i].refToFrame);
			trackPermaRef.update( time );
		}

		float score = getRefFrameScore(potentialReferenceFrames[i].dist, tracker->pointUsage);

		if(score < maxScore)
		{
			SE3 RefToFrame_tracked = tracker->trackFrameOnPermaref(potentialReferenceFrames[i].keyframe, keyframe->frame(), potentialReferenceFrames[i].refToFrame);
			Sophus::Vector3d dist = RefToFrame_tracked.translation() * potentialReferenceFrames[i].keyframe->frame()->meanIdepth;

			float newScore = getRefFrameScore(dist.dot(dist), tracker->pointUsage);
			float poseDiscrepancy = (potentialReferenceFrames[i].refToFrame * RefToFrame_tracked.inverse()).log().norm();
			float goodVal = tracker->pointUsage * tracker->lastGoodCount() / (tracker->lastGoodCount()+tracker->lastBadCount());
			checkedSecondary++;

			if(tracker->_trackingWasGood && goodVal > relocalizationTH && newScore < bestScore && poseDiscrepancy < 0.2)
			{
				bestPoseDiscrepancy = poseDiscrepancy;
				bestScore = score;
				bestFrame = potentialReferenceFrames[i].keyframe;
				bestRefToFrame = potentialReferenceFrames[i].refToFrame;
				bestRefToFrame_tracked = RefToFrame_tracked;
				bestDist = dist.dot(dist);
				bestUsage = tracker->pointUsage;
			}
		}
	}

	if(bestFrame != 0)
	{
		if(Conf().print.relocalizationInfo)
			printf("FindReferences for %d: Checked %d (%d). dist %.3f + usage %.3f = %.3f. pose discrepancy %.2f. TAKE %d!\n",
					(int)keyframe->id(), (int)potentialReferenceFrames.size(), checkedSecondary,
					bestDist, bestUsage, bestScore,
					bestPoseDiscrepancy, bestFrame->id());
		return bestFrame;
	}
	else
	{
		if(Conf().print.relocalizationInfo)
			printf("FindReferences for %d: Checked %d (%d), bestScore %.2f. MAKE NEW\n",
					(int)keyframe->id(), (int)potentialReferenceFrames.size(), checkedSecondary, bestScore);
		return 0;
	}
}

std::unordered_set<KeyFrame::SharedPtr> TrackableKeyFrameSearch::findCandidates(const KeyFrame::SharedPtr &keyframe, KeyFrame::SharedPtr &fabMapResult_out, bool includeFABMAP, bool closenessTH)
{
	std::unordered_set<KeyFrame::SharedPtr> results;

	// Add all candidates that are similar in an euclidean sense.
	std::vector<TrackableKFStruct> potentialReferenceFrames = findEuclideanOverlapFrames(keyframe, closenessTH * 15 / (KFDistWeight*KFDistWeight), 1.0 - 0.25 * closenessTH, true);
	for(unsigned int i=0;i<potentialReferenceFrames.size();i++)
		results.insert(potentialReferenceFrames[i].keyframe);

	int appearanceBased = 0;
	fabMapResult_out = 0;
	if(includeFABMAP)
	{
		// Add Appearance-based Candidate, and all it's neighbours.
		fabMapResult_out = findAppearanceBasedCandidate(keyframe);
		if(fabMapResult_out != nullptr)
		{
			results.insert(fabMapResult_out);
			results.insert(fabMapResult_out->neighbors.begin(), fabMapResult_out->neighbors.end());
			appearanceBased = 1 + fabMapResult_out->neighbors.size();
		}
	}

	LOGF_IF(DEBUG, Conf().print.constraintSearchInfo, "Early LoopClosure-Candidates for %d: %d euclidean, %d appearance-based, %d total\n",
				(int)keyframe->id(), (int)potentialReferenceFrames.size(), appearanceBased, (int)results.size());

	return results;
}

KeyFrame::SharedPtr TrackableKeyFrameSearch::findAppearanceBasedCandidate( const KeyFrame::SharedPtr &keyframe)
{
#ifdef HAVE_FABMAP
	if(!useFabMap) return nullptr;


	if (! fabMap.isValid())
	{
		printf("Error: called findAppearanceBasedCandidate(), but FabMap instance is not valid!\n");
		return nullptr;
	}


	int newID, loopID;
	fabMap.compareAndAdd(keyframe, &newID, &loopID);
	if (newID < 0)
		return nullptr;

	fabmapIDToKeyframe.insert(std::make_pair(newID, keyframe));
	if (loopID >= 0)
		return fabmapIDToKeyframe.at(loopID);
	else
		return nullptr;
#else
	LOG_IF(WARNING, useFabMap) << "Warning: Compiled without FabMap, but useFabMap is enabled... ignoring.";
	return nullptr;
#endif
}


}
