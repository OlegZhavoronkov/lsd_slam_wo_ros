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
            auto f=*(graph->_keyFrames.rbegin());
             //calculate PointCloud
            
            float param[4]{f->frame()->fx(0), f->frame()->fy(0), f->frame()->cx(0), f->frame()->cy(0)};
            float fxi = 1/param[0];
            float fyi = 1/param[1];
            float cxi = -param[2] / param[0];
            float cyi = -param[3] / param[1]; //inverse cam param
    int w = f->width(0), h = f->height(0);
    auto camToWorld = f->pose->getCamToWorld();
    float my_scaledTH = 4e-3, my_scale = camToWorld.scale(), my_absTH = 4e-2;
    int my_minNearSupport = 7;
    const float* idepth = f->idepth(0);
    const float* idepthVar = f->idepthVar(0);
    const float* color = f->image(0);
    //go through all points
    int new_sz = 0;
    pts.reserve(w * h);
	const float* colorRGB = f->color();


	Eigen::Matrix<float, 3, 4> PC;
	PC << cp11, cp12, cp13, cp14,
	      cp21, cp22, cp23, cp24,
	      cp31, cp32, cp33, cp34;
	int nocolor = 0, oob = 0;
    for (int y = 1; y + 1 < h; ++y)
        for (int x = 1; x + 1 < w; ++x)
        {
            if (idepth[x + y * w] <= 0.f)
                continue;
            float depth = 1 / idepth[x + y * w];
            float depth4 = depth*depth;
            depth4*= depth4;
            if(idepthVar[x+y*w] * depth4 > my_scaledTH)
                continue;

            if(idepthVar[x+y*w] * depth4 * my_scale*my_scale > my_absTH)
                continue;

            if(my_minNearSupport > 1)
            {
                int nearSupport = 0;
                for(int dx=-1;dx<2;dx++)
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

                if(nearSupport < my_minNearSupport)
                    continue;
            }
            //coord of point relative to the KF
            MyVertex a;
#if 1
            a.point = Eigen::Vector3f(x * fxi + cxi, y * fyi + cyi, 1.f) * depth;
#else
            a.point = qglviewer::Vec(x * fxi + cxi, y * fyi + cyi, 1.f) * depth;
#endif
		bool colorValid = false;
		if (colorRGB)
		{
			Eigen::Vector3f pt(x * fxi + cxi, y * fyi + cyi, 1.0f);
			pt *= depth;
			pt *= camToWorld.scale() * globalScaler;
			Eigen::Vector4f ptw(pt[0], pt[1], pt[2], 1.0f);
			Eigen::Vector3f proj = PC * ptw;
			proj *= 1.0 / proj[2];
			if (proj[0] < cpw && proj[0] >= 0 && proj[1] < cph && proj[1] >= 0)
			{
				int cx = proj[0], cy = proj[1];
				int cidx = cx * 3 + cy * cpw * 3;
				auto ptr = colorRGB + cidx;
				a.b = ptr[0] / 255.f;
				a.g = ptr[1] / 255.f;
				a.r = ptr[2] / 255.f;
				colorValid = true;
			}
			else
				oob++;
		}
		else
			nocolor++;
		if (!colorValid)
            a.r = a.g = a.b = color[x + y * w] / 255.f;
            if (new_sz == pts.size())
                pts.push_back(a);
            pts[new_sz++] = a;
        }
    pts.resize(new_sz);
        }
        _lastKFId=KFId;
    }
}

}