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

#include "sophus/se3.hpp"
#include "sophus/sim3.hpp"
#include "SophusUtil.h"
#include <fmt/format.h>
// Compile the templates here once so they don't need to be compiled in every
// other file using them.
// 
// Other files then include SophusUtil.h which contains extern template
// declarations to prevent compiling them there again. (For this reason,
// this header must not be included here).
//
// Eigen::Matrix seemingly cannot be instantiated this way, as it tries to
// compile a constructor variant for 4-component vectors, resulting in a
// static assertion failure.


template class Eigen::Quaternion<float>;
template class Eigen::Quaternion<double>;

template class Sophus::SE3Group<float, 0>;
template class Sophus::SE3Group<double, 0>;

template class Sophus::Sim3Group<float, 0>;
template class Sophus::Sim3Group<double, 0>;

Eigen::Vector3d QuatToEulers(const Eigen::Quaternion<double> & q)
{
    Eigen::Vector3d ret={};
    double w=q.w();
    double x=q.x();
    double y=q.y();
    double z=q.z();

    double sinr_cosp = 2 * (w*  x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    ret.x()= std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (w * y - x * z));
    double cosp = std::sqrt(1 - 2 * (w * y - x * z));
    ret.y()= 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    ret.z()=std::atan2(siny_cosp, cosy_cosp);
    return ret;
}

Eigen::Vector3f QuatToEulers(const Eigen::Quaternion<float> & q)
{
    Eigen::Vector3f ret={};
    float w=q.w();
    float x=q.x();
    float y=q.y();
    float z=q.z();

    float sinr_cosp = 2 * (w*  x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    ret.x()= std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = std::sqrt(1 + 2 * (w * y - x * z));
    float cosp = std::sqrt(1 - 2 * (w * y - x * z));
    ret.y()= 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    ret.z()=std::atan2(siny_cosp, cosy_cosp);
    return ret;
}

std::ostream& operator<<(std::ostream& s,const Sim3& sim3)
{
    const auto& tr=sim3.translation();
    const auto& quat=sim3.quaternion();
    auto eulers=QuatToEulers( quat)*(180/M_PI);
    return s<< fmt::format("\tsim3 tr: \t[{0:.6f} {1:.6f} {2:.6f}]\n\tquat [{3:.6f} {4:.6f} {5:.6f} {6:.6f}] quat norm {7:.6f}\n\teulers [{8:.3f} {9:.3f} {10:.3f}]",
                                tr.x(),tr.y(),tr.z(),
                                quat.x(),quat.y(),quat.z(),quat.w(),
                                quat.norm(),
                                eulers.x(),eulers.y(),eulers.z()
                            );
}
std::ostream& operator<<(std::ostream& s,const SO3& so3)
{
    const auto& quat=so3.unit_quaternion();
    auto eulers=QuatToEulers( quat)*(180/M_PI);
    return s<< fmt::format("\tso3 quat [{0:.6f} {1:.6f} {2:.6f} {3:.6f}] quat norm {4:.6f}\n\teulers [{5:.3f} {6:.3f} {7:.3f}]",
                                quat.x(),quat.y(),quat.z(),quat.w(),
                                quat.norm(),
                                eulers.x(),eulers.y(),eulers.z()
                            );
}
std::ostream& operator<<(std::ostream& s,const SE3& se3)
{
    const auto& tr=se3.translation();
    const auto& quat=se3.unit_quaternion();
    auto eulers=QuatToEulers( quat)*(180/M_PI);
    return s<< fmt::format("\tse3 tr: [{0:.6f} {1:.6f} {2:.6f}]\n\tquat [{3:.6f} {4:.6f} {5:.6f} {6:.6f}] quat norm {7:.6f}\n\teulers [{8:.3f} {9:.3f} {10:.3f}]",
                                tr.x(),tr.y(),tr.z(),
                                quat.x(),quat.y(),quat.z(),quat.w(),
                                quat.norm(),
                                eulers.x(),eulers.y(),eulers.z()
                            );
}

std::ostream& operator<<(std::ostream& s,const Sophus::SE3f& se3)
{
    const auto& tr=se3.translation();
    const auto& quat=se3.unit_quaternion();
    auto eulers=QuatToEulers( quat)*(180/M_PI);
    return s<< fmt::format("\tse3 tr: [{0:.6f} {1:.6f} {2:.6f}]\n\tquat [{3:.6f} {4:.6f} {5:.6f} {6:.6f}] quat norm {7:.6f}\n\teulers [{8:.3f} {9:.3f} {10:.3f}]",
                                tr.x(),tr.y(),tr.z(),
                                quat.x(),quat.y(),quat.z(),quat.w(),
                                quat.norm(),
                                eulers.x(),eulers.y(),eulers.z()
                            );
}
