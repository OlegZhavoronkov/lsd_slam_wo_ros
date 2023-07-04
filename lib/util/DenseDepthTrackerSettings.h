

#pragma once


namespace lsd_slam {

  template <int __LEVELS>
  class DenseDepthTrackerSettings
  {
  public:
  	DenseDepthTrackerSettings()
  	{
  		// Set default settings
  		if (__LEVELS > 6)
  			printf("WARNING: Sim3Tracker(): default settings are intended for a maximum of 6 levels!");

  		lambdaSuccessFac = 0.5f;
  		lambdaFailFac = 2.0f;

  		const float stepSizeMinc[6] = {1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8};
  		const int maxIterations[6] = {5, 20, 50, 100, 100, 100};


  		for (int level = 0; level < __LEVELS; ++ level)
  		{
  			lambdaInitial[level] = 0;
  			stepSizeMin[level] = stepSizeMinc[level];
  			convergenceEps[level] = 0.999f;
  			maxItsPerLvl[level] = maxIterations[level];
  		}

  		lambdaInitialTestTrack = 0;
  		stepSizeMinTestTrack = 1e-3;
  		convergenceEpsTestTrack = 0.98;
  		maxItsTestTrack = 5;

  		var_weight = 1.0;
  		huber_d = 3;
  	}

  	float lambdaSuccessFac;
  	float lambdaFailFac;
  	float lambdaInitial[__LEVELS];
  	float stepSizeMin[__LEVELS];
  	float convergenceEps[__LEVELS];
  	int maxItsPerLvl[__LEVELS];

  	float lambdaInitialTestTrack;
  	float stepSizeMinTestTrack;
  	float convergenceEpsTestTrack;
  	float maxItsTestTrack;

  	float huber_d;
  	float var_weight;
  };

}
