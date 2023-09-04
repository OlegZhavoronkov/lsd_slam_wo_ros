#pragma once
#include <string>
#include "IOWrapper/OutputIOWrapper.h"
namespace lsd_slam::PlyOutputWrapper
{

class MeshOutputWrapper:public OutputIOWrapper
{
public:
    MeshOutputWrapper()=delete;
    MeshOutputWrapper(const std::string& path);
    virtual void publishPose( const Sophus::Sim3f &pose ) override{};

	virtual void publishKeyframeGraph( const std::shared_ptr<KeyFrameGraph> &graph) override;

	virtual void publishPointCloud( const Frame::SharedPtr &kf ) override{};

	// publishes a keyframe. if that frame already exists, it is overwritten, otherwise it is added.
	virtual void publishKeyframe(const Frame::SharedPtr &kf) override{};

	virtual void updateDepthImage(unsigned char * data) override {};

	// published a tracked frame that did not become a keyframe (yet; i.e. has no depth data)
	virtual void publishTrackedFrame(const Frame::SharedPtr &kf) override{};

	// publishes graph and all constraints, as well as updated KF poses.
	virtual void publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier) override{};
	virtual void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier) override{};

	virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data) override{};

	virtual void updateFrameNumber( int ) override{};
	virtual void updateLiveImage( const cv::Mat &img ) override{};

    struct IPlyWrapper
    {
        virtual ~IPlyWrapper()=0;
        virtual void DumpToStream()=0;
    };
private:
    std::string _path;
    std::shared_ptr<IPlyWrapper> CreateFile();
    int _lastKFId,_prevDumpedKF;
    std::shared_ptr<IPlyWrapper> _pPly;
    std::string _folder;
};

}