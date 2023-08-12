#include "MeshOutputWrapper.h"

namespace lsd_slam
{

MeshOutputWrapper::MeshOutputWrapper(const std::string& path)
    :   _path(path),
        _lastKFId(-1)
{

}

void MeshOutputWrapper::publishKeyframeGraph( const std::shared_ptr<KeyFrameGraph> &graph)
{
    struct MyVertex
    {
        Eigen::Vector3f point;
    };
    if(graph->_keyFrames.empty())
    {
        return;
    }
    auto KFId=(*(graph->_keyFrames.rbegin()))->id();
    if(KFId<=_lastKFId)
    {
        return;
    }
    else
    {
        if(_lastKFId < 0)
        {
            CreateFile();
        }
        {
            std::vector<MyVertex> pts;
            auto keyf=*(graph->_keyFrames.rbegin());
            auto frame=keyf->frame();
             //calculate PointCloud
            
            float param[4]{frame->fx(0), frame->fy(0), frame->cx(0), frame->cy(0)};
            float fxi = 1/param[0];
            float fyi = 1/param[1];
            float cxi = -param[2] / param[0];
            float cyi = -param[3] / param[1]; //inverse cam param
            auto frame_size = frame->imgSize(0).cvSize();
            int h = frame_size.height;
            int w = frame_size.width;
            auto camToWorld = frame->pose->getCamToWorld();
            float my_scaledTH = 4e-3, my_scale = camToWorld.scale(), my_absTH = 4e-2;
            int my_minNearSupport = 7;
            const float* idepth = frame->idepth(0);
            const float* idepthVar = frame->idepthVar(0);
    
            //go through all points
            int new_sz = 0;
            pts.reserve(w * h);
	        


	
        	//int nocolor = 0, oob = 0;
            for (int y = 1; y + 1 < h; ++y)
            {
                for (int x = 1; x + 1 < w; ++x)
                {
                    if (idepth[x + y * w] <= 0.f)
                    {
                        continue;
                    }
                    float depth = 1 / idepth[x + y * w];
                    float depth4 = depth*depth;
                    depth4*= depth4;
                    if(idepthVar[x+y*w] * depth4 > my_scaledTH)
                    {
                        continue;
                    }

                    if(idepthVar[x+y*w] * depth4 * my_scale*my_scale > my_absTH)
                    {
                        continue;
                    }

                    if(my_minNearSupport > 1)
                    {
                        int nearSupport = 0;
                        for(int dx=-1;dx<2;dx++)
                        {
                            for(int dy=-1;dy<2;dy++)
                            {
                                int idx = x+dx+(y+dy)*w;
                                if(idepth[idx] > 0)
                                {
                                    float diff = idepth[idx] - 1.0f / depth;
                                    if(diff*diff < 2*idepthVar[x+y*w])
                                        nearSupport++;
                                }
                            }
                        }

                        if(nearSupport < my_minNearSupport)
                        {
                            continue;
                        }
                    }
                    //coord of point relative to the KF
                    MyVertex a;
                    a.point = (camToWorld* Eigen::Vector3d(x * fxi + cxi, y * fyi + cyi, 1.f) * depth).cast<float>();
                    pts.emplace_back(std::move(a));
                    new_sz++;
                }
            }
            pts.resize(new_sz);
        }
        _lastKFId=KFId;
    }
}

}