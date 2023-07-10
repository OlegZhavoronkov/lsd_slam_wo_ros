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
#include <thread>
#include <stdio.h>
#include <iostream>
#include <condition_variable>



namespace lsd_slam
{

class IndexThreadReduce
{

public:
	inline IndexThreadReduce()
	{
		nextIndex = 0;
		maxIndex = 0;
		stepSize = 1;
		callPerIndex = std::bind_front(&IndexThreadReduce::callPerIndexDefault, this);

		running = true;
		for(int i=0;i<MAPPING_THREADS;i++)
		{
			isDone[i] = false;
			workerThreads[i] = std::thread(&IndexThreadReduce::workerLoop, this, i);
		}

		//printf("created ThreadReduce\n");
	}
	inline ~IndexThreadReduce()
	{
		running = false;

		exMutex.lock();
		todo_signal.notify_all();
		exMutex.unlock();

		for(int i=0;i<MAPPING_THREADS;i++)
			workerThreads[i].join();


		//printf("destroyed ThreadReduce\n");

	}

	inline void reduce(std::function<void(int,int,RunningStats*)> callPerIndex, int first, int end, int stepSize = 0)
	{
		if(!multiThreading)
		{
			callPerIndex(first, end, &runningStats);
			return;
		}



		if(stepSize == 0)
			stepSize = ((end-first)+MAPPING_THREADS-1)/MAPPING_THREADS;


		//printf("reduce called\n");

		std::unique_lock<std::mutex> lock(exMutex);

		// save
		this->callPerIndex = callPerIndex;
		nextIndex = first;
		maxIndex = end;
		this->stepSize = stepSize;

		// go worker threads!
		for(int i=0;i<MAPPING_THREADS;i++)
			isDone[i] = false;

		// let them start!
		todo_signal.notify_all();


		//printf("reduce waiting for threads to finish\n");
		// wait for all worker threads to signal they are done.
		while(true)
		{
			// wait for at least one to finish
			done_signal.wait(lock);
			//printf("thread finished!\n");

			// check if actually all are finished.
			bool allDone = true;
			for(int i=0;i<MAPPING_THREADS;i++)
				allDone = allDone && isDone[i];

			// all are finished! exit.
			if(allDone)
				break;
		}

		nextIndex = 0;
		maxIndex = 0;
		this->callPerIndex = std::bind_front(&IndexThreadReduce::callPerIndexDefault, this);

		//printf("reduce done (all threads finished)\n");
	}


private:
	std::thread workerThreads[MAPPING_THREADS];
	bool isDone[MAPPING_THREADS];

	std::mutex exMutex;
	std::condition_variable todo_signal;
	std::condition_variable done_signal;

	int nextIndex;
	int maxIndex;
	int stepSize;

	bool running;

	std::function<void(int,int,RunningStats*)> callPerIndex;

	void callPerIndexDefault(int i, int j,RunningStats* k)
	{
		printf("ERROR: should never be called....\n");
	}

	void workerLoop(int idx)
	{
		std::unique_lock<std::mutex> lock(exMutex);

		while(running)
		{
			// try to get something to do.
			int todo = 0;
			bool gotSomething = false;
			if(nextIndex < maxIndex)
			{
				// got something!
				todo = nextIndex;
				nextIndex+=stepSize;
				gotSomething = true;
			}

			// if got something: do it (unlock in the meantime)
			if(gotSomething)
			{
				lock.unlock();

				assert(callPerIndex != 0);

				RunningStats s;
				callPerIndex(todo, std::min(todo+stepSize, maxIndex), &s);

				lock.lock();
				runningStats.add(&s);
			}

			// otherwise wait on signal, releasing lock in the meantime.
			else
			{
				isDone[idx] = true;
				//printf("worker %d waiting..\n", idx);
				done_signal.notify_all();
				todo_signal.wait(lock);
			}
		}
	}
};
}
