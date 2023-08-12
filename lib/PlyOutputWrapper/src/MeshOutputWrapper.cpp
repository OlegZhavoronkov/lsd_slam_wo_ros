#include <PlyOutputWrapper/MeshOutputWrapper.h>
#include <tinyply.h>
#include <fmt/format.h>

namespace lsd_slam::PlyOutputWrapper
{

class RealPlyWriter:public MeshOutputWrapper::IPlyWrapper
{
public:
    RealPlyWriter(const std::string& path)
        : _path(path)
    {

    }
    RealPlyWriter()=delete;
    RealPlyWriter(const RealPlyWriter&)=delete;
    RealPlyWriter& operator=(const RealPlyWriter&)=delete;
    virtual ~RealPlyWriter()=default;

    virtual void DumpToStream() override
    {
            std::filebuf fb_ascii;
            fb_ascii.open(_path, std::ios::out | std::ios::trunc);
            std::ostream outstream_ascii(&fb_ascii);
            if (outstream_ascii.fail()) throw std::runtime_error("failed to open " + _path);

            tinyply::PlyFile file;

            file.add_properties_to_element("vertex", { "x",  }, 
                tinyply::Type::FLOAT32, _x.size(), reinterpret_cast<uint8_t*>(_x.data()), tinyply::Type::INVALID, 0);
            file.add_properties_to_element("vertex", { "y",  }, 
                tinyply::Type::FLOAT32, _y.size(), reinterpret_cast<uint8_t*>(_y.data()), tinyply::Type::INVALID, 0);
            file.add_properties_to_element("vertex", { "z",  }, 
                tinyply::Type::FLOAT32, _z.size(), reinterpret_cast<uint8_t*>(_z.data()), tinyply::Type::INVALID, 0);


            file.get_comments().push_back("generated by lsd slam");
            file.write(outstream_ascii, false);
    }
    void AddVerticesBuffer(size_t pointsNum,const std::vector<float>& data)
    {
        if(pointsNum*3 != data.size())
        {
            throw std::runtime_error(fmt::v7::format("pointsnum {0} != data.size() {1} /3 {2}",pointsNum,data.size(),data.size()/3));
        }
        std::vector<float> new_x;
        new_x.reserve(_x.size()+pointsNum);
        std::copy(_x.begin(),_x.end(),new_x.begin());
        std::vector<float> new_y;
        new_y.reserve(_y.size()+pointsNum);
        std::copy(_y.begin(),_y.end(),new_y.begin());
        std::vector<float> new_z;
        new_z.reserve(_z.size()+pointsNum);
        std::copy(_z.begin(),_z.end(),new_z.begin());
        for(size_t idx =0;idx<pointsNum;idx+=3)
        {
            const float* pPoint= &(data.at(idx));
            new_x.push_back(pPoint[0]);
            new_y.push_back(pPoint[1]);
            new_z.push_back(pPoint[2]);
        }
        _x.swap(new_x);
        _y.swap(new_y);
        _z.swap(new_z);
    }
private:
    std::vector<float> _x,_y,_z;
    std::string _path;
};

MeshOutputWrapper::IPlyWrapper::~IPlyWrapper()=default;

MeshOutputWrapper::MeshOutputWrapper(const std::string& path)
    :   _path(path),
        _lastKFId(-1),
        _prevDumpedKF(0)
{

}

std::shared_ptr<MeshOutputWrapper::IPlyWrapper> MeshOutputWrapper::CreateFile()
{
    return std::make_shared<RealPlyWriter>(_path);
}

void MeshOutputWrapper::publishKeyframeGraph( const std::shared_ptr<KeyFrameGraph> &graph)
{
    //struct MyVertex
    //{
    //    Eigen::Vector3f point;
    //};
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
            _pPly= CreateFile();
        }
        {
            std::vector<float> pts;
            size_t points_num=0;
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
            pts.reserve(3*w * h);
	        


	
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
                    
                    Eigen::Vector3f point = (camToWorld* Eigen::Vector3d(x * fxi + cxi, y * fyi + cyi, 1.f) * depth).cast<float>();
                    pts.emplace_back(point.x());pts.emplace_back(point.y());pts.emplace_back(point.z());
                    new_sz+=3;
                    points_num++;
                }
            }
            pts.resize(new_sz);
            auto localPly=_pPly;
            if(localPly)
            {
                RealPlyWriter* pRealPly=static_cast<RealPlyWriter*>(localPly.get());
                pRealPly->AddVerticesBuffer(points_num,pts);
                if((_lastKFId-_prevDumpedKF)>20)
                {
                    pRealPly->DumpToStream();
                    _prevDumpedKF=_lastKFId;
                }
            }
        }
        _lastKFId=KFId;
    }
}

}