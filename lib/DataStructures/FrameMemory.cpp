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

#include "DataStructures/FrameMemory.h"
#include "DataStructures/Frame.h"

namespace lsd_slam
{

FrameMemory::FrameMemory()
{
}

FrameMemory& FrameMemory::getInstance()
{
    std::scoped_lock lock(_instanceMtx);
	static FrameMemory theOneAndOnly;
	return theOneAndOnly;
}

void FrameMemory::releaseBuffers()
{
	std::unique_lock<std::mutex> lock(_accessMutex);
	int total = 0;


	for(auto p : _availableBuffers)
	{
		LOGF_IF(DEBUG, Conf().print.memoryDebugInfo, "deleting %d buffers of size %d!", (int)p.second.size(), (int)p.first);

		total += p.second.size() * p.first;

		for(unsigned int i=0;i<p.second.size();i++)
		{
			delete[] (char*)p.second[i];
			_bufferSizes.erase(p.second[i]);
		}

		p.second.clear();
	}
	_availableBuffers.clear();

	LOGF_IF(DEBUG, Conf().print.memoryDebugInfo, "released %.1f MB!", total / (1000000.0f));
}


void* FrameMemory::getBuffer(unsigned int sizeInByte)
{
	std::unique_lock<std::mutex> lock(_accessMutex);

	if (_availableBuffers.count(sizeInByte) > 0)
	{
		std::vector< void* >& availableOfSize = _availableBuffers.at(sizeInByte);
		if (availableOfSize.empty())
		{
			void* buffer = allocateBuffer(sizeInByte);
//			assert(buffer != 0);
			return buffer;
		}
		else
		{
			void* buffer = availableOfSize.back();
			availableOfSize.pop_back();

//			assert(buffer != 0);
			return buffer;
		}
	}
	else
	{
		void* buffer = allocateBuffer(sizeInByte);
//		assert(buffer != 0);
		return buffer;
	}
}

float* FrameMemory::getFloatBuffer(unsigned int size)
{
	return (float*)getBuffer(sizeof(float) * size);
}

void FrameMemory::returnBuffer(void* buffer)
{
	if(buffer==nullptr) return;

	std::unique_lock<std::mutex> lock(_accessMutex);

	unsigned int size = _bufferSizes.at(buffer);
	//printf("returnFloatBuffer(%d)\n", size);
	if (_availableBuffers.count(size) > 0)
		_availableBuffers.at(size).push_back(buffer);
	else
	{
		std::vector< void* > availableOfSize;
		availableOfSize.push_back(buffer);
		_availableBuffers.insert(std::make_pair(size, availableOfSize));
	}
}

void* FrameMemory::allocateBuffer(unsigned int size)
{
	void* buffer = (void*)(new char[size]);
	LOG_IF(DEBUG, Conf().print.memoryDebugInfo) << "Alloc " << size << " at " << std::ios::hex << buffer;
	_bufferSizes.insert(std::make_pair(buffer, size));
	return buffer;
}

std::shared_lock<std::shared_timed_mutex> FrameMemory::activateFrame(Frame* frame)
{
	std::unique_lock<std::mutex> lock(_activeFramesMutex);
	if(frame->isActive)
		_activeFrames.remove(frame);
	_activeFrames.push_front(frame);
	frame->isActive = true;
	return std::shared_lock<std::shared_timed_mutex>(frame->activeMutex);
}

void FrameMemory::deactivateFrame(Frame* frame)
{
	std::unique_lock<std::mutex> lock(_activeFramesMutex);
	if(!frame->isActive) return;
	_activeFrames.remove(frame);

	while(!frame->minimizeInMemory())
		LOG(WARNING) << "cannot deactivateFrame frame " << frame->id()
								<< ", as some active locks are lingering. May cause deadlock!";	// do it in a loop, to make sure it is really, really deactivated.

	frame->isActive = false;
}

void FrameMemory::pruneActiveFrames()
{
	std::unique_lock<std::mutex> lock(_activeFramesMutex);

	while((int)_activeFrames.size() > maxLoopClosureCandidates + 20)
	{
		if(!_activeFrames.back()->minimizeInMemory())
		{
			if(!_activeFrames.back()->minimizeInMemory())
			{
				LOG(WARNING) << "failed to minimize frame " << _activeFrames.back()->id() << " twice. maybe some active-lock is lingering?";
				return;	 // pre-emptive return if could not deactivate.
			}
		}
		_activeFrames.back()->isActive = false;
		_activeFrames.pop_back();
	}
}

 std::mutex FrameMemory::_instanceMtx;

}
