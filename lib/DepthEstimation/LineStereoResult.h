#pragma once

namespace lsd_slam
{

enum struct LineStereoResult:int
{
    OK_ERROR_IN_BOUND =         1,//cv::Vec3b(255,255,255)); // white for GOT CREATED
    OK                  =OK_ERROR_IN_BOUND,
    FAIL_UNKNOWN               =       0,
    OUT_OF_BOUNDS =             -1,//cv::Vec3b(0,0,255));       // RED FOR OOB
    NOT_GOOD_FOR_STEREO =       -2,//cv::Vec3b(255,0,255));     // PURPLE FOR NON-GOOD
    NOT_FOUND_ERROR_TOO_HIGH =  -3,//cv::Vec3b(0,0,0)); // BLACK FOR big not-found
    BIG_NOT_FOUND            =  -4,//cv::Vec3b(0,0,0)); // BLACK FOR big not-found
    BIG_INCONSISTENT         = -5,//cv::Vec3b(255,255,0));     // Turkoise FOR big inconsistent
    SKIPPED_NOT_GOOD_TRACKED = -6,//cv::Vec3b(255,0,0));  // BLUE for SKIPPED NOT GOOD TRACKED
    VARIANCE_LARGER_THAN_INITIAL_HYPOTHESIS_REMOVED = -7,
    CALCULATION_EPL_FAILED_DUE_TO_INFS = -8,
    CALCULATION_EPL_FAILED_EPL_LENGTH_TOO_SMALL = -9,
    CALCULATION_EPL_FAILED_EPL_GRAD_TOO_SMALL = -10,
    CALCULATION_EPL_FAILED_EPL_ANGLE_TOO_SMALL = -11
    
};

inline bool operator==(LineStereoResult res,bool val)
{
    return val ? 
                    res==LineStereoResult::OK :
                    res!=LineStereoResult::OK ;
}

inline bool operator!(LineStereoResult res)
{
    return !(res == LineStereoResult::OK);
}

}