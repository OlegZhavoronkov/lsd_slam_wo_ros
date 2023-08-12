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

#include "util/globalFuncs.h"
#include "util/SophusUtil.h"
#include "opencv2/opencv.hpp"
#include "DataStructures/Frame.h"
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/replace.hpp>
#if defined(__linux__) || defined(__unix__) || defined(__ANDROID__) || defined(_POSIX_VERSION)
#include <signal.h>
#define USE_SIGNAL
#elif defined(WIN32) || defined(__WIN32__) || defined(__NT__)
#include 
#define USE_WIN
#endif

namespace lsd_slam
{

ThreadSynchronizer startAll;

SE3 SE3CV2Sophus(const cv::Mat &R, const cv::Mat &t)
{
	Sophus::Matrix3f sR;
	Sophus::Vector3f st;

	for(int i=0;i<3;i++)
	{
		sR(0,i) = R.at<double>(0,i);
		sR(1,i) = R.at<double>(1,i);
		sR(2,i) = R.at<double>(2,i);
		st[i] = t.at<double>(i);
	}

	return SE3(toSophus(sR.inverse()), toSophus(st));
}

void printMessageOnCVImage(cv::Mat &image, std::string line1,std::string line2)
{
	for(int x=0;x<image.cols;x++)
		for(int y=image.rows-30; y<image.rows;y++)
			image.at<cv::Vec3b>(y,x) *= 0.5;

	cv::putText(image, line2, cv::Point(10,image.rows-5),
	    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200,200,250), 1, 8);

	cv::putText(image, line1, cv::Point(10,image.rows-18),
	    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200,200,250), 1, 8);
}


cv::Mat getDepthRainbowPlot(Frame* kf, int lvl)
{
	return getDepthRainbowPlot(kf->idepth(lvl), kf->idepthVar(lvl), kf->image(lvl),
			kf->width(lvl), kf->height(lvl));
}

cv::Mat getDepthRainbowPlot(const float* idepth, const float* idepthVar, const float* gray, int width, int height)
{
	cv::Mat res = cv::Mat(height,width,CV_8UC3);
	if(gray != 0)
	{
		cv::Mat keyFrameImage(height, width, CV_32F, const_cast<float*>(gray));
		cv::Mat keyFrameImage8u;
		keyFrameImage.convertTo(keyFrameImage8u, CV_8UC1);
		cv::cvtColor(keyFrameImage8u, res, cv::COLOR_GRAY2RGB);
	}
	else
		fillCvMat(&res,cv::Vec3b(255,170,168));
    float min_val=std::numeric_limits<float>::max();
    float max_val=std::numeric_limits<float>::min();
    for(int i=0;i<width;i++)
    {
		for(int j=0;j<height;j++)
		{
			float id = idepth[i + j*width];

			if(id >=0 && idepthVar[i + j*width] >= 0)
            {
                min_val = id < min_val ? id :min_val;
                max_val = id > max_val ? id :max_val;
            }
        }
    }

	for(int i=0;i<width;i++)
		for(int j=0;j<height;j++)
		{
			float id = idepth[i + j*width];

			if(id >=0 && idepthVar[i + j*width] >= 0)
			{
                float id1= 3*(id-min_val)/(max_val-min_val);
				// rainbow between 0 and 4
				float r = (0-id1) * 255 / 1.0; if(r < 0) r = -r;
				float g = (1-id1) * 255 / 1.0; if(g < 0) g = -g;
				float b = (2-id1) * 255 / 1.0; if(b < 0) b = -b;

				uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
				uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
				uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);

				res.at<cv::Vec3b>(j,i) = cv::Vec3b(255-rc,255-gc,255-bc);
			}
		}
	return res;
}
cv::Mat getVarRedGreenPlot(const float* idepthVar, const float* gray, int width, int height)
{
    float* idepthVarExt = new float[width*height];
	memcpy(idepthVarExt,idepthVar,sizeof(float)*width*height);

	for(int i=2;i<width-2;i++)
		for(int j=2;j<height-2;j++)
		{
			if(idepthVar[(i) + width*(j)] <= 0)
				idepthVarExt[(i) + width*(j)] = -1;
			else
			{
				float sumIvar = 0;
				float numIvar = 0;
				for(int dx=-2; dx <=2; dx++)
					for(int dy=-2; dy <=2; dy++)
					{
						if(idepthVar[(i+dx) + width*(j+dy)] > 0)
						{
							float distFac = (float)(dx*dx+dy*dy)*(0.075*0.075)*0.02;
							float ivar = 1.0f/(idepthVar[(i+dx) + width*(j+dy)] + distFac);
							sumIvar += ivar;
							numIvar += 1;
						}
					}
				idepthVarExt[(i) + width*(j)] = numIvar / sumIvar;
			}

		}


	cv::Mat res = cv::Mat(height,width,CV_8UC3);
	if(gray != 0)
	{
		cv::Mat keyFrameImage(height, width, CV_32F, const_cast<float*>(gray));
		cv::Mat keyFrameImage8u;
		keyFrameImage.convertTo(keyFrameImage8u, CV_8UC1);
		cv::cvtColor(keyFrameImage8u, res, cv::COLOR_GRAY2RGB);
	}
	else
		fillCvMat(&res,cv::Vec3b(255,170,168));

	for(int i=0;i<width;i++)
		for(int j=0;j<height;j++)
		{
			float idv = idepthVarExt[i + j*width];

			if(idv > 0)
			{
				float var= sqrt(idv);

				var = var*60*255*0.5 - 20;
				if(var > 255) var = 255;
				if(var < 0) var = 0;

				res.at<cv::Vec3b>(j,i) = cv::Vec3b(0,255-var, var);
			}
		}

	delete[] idepthVarExt;
	return res;
}

template<> void DebugImage<float>(const std::string& debugMsg,const float* data,int height,int width)
{
    std::unique_ptr<float[]> pbuff(new float[width*height]);
    memcpy(pbuff.get(),data,width*height*sizeof(float));
    {

    
    //cv::Mat mat=cv::Mat::zeros(cv::Size(width,height),CV_32FC1,);// (height,width,CV_32F,(void*)reinterpret_cast<const void*>(data),);
    cv::Mat mat(cv::Size(width,height),CV_32FC1,pbuff.get());// (height,width,CV_32F,(void*)reinterpret_cast<const void*>(data),);
    //void* pData= mat.ptr(0);
    
    //memcpy(pData,data,width*height*2);
    cv::Mat mat2;
    mat.convertTo(mat2,CV_8UC1);
    cv::imshow(debugMsg,mat2);
    cv::waitKey(1);
    }
}

cv::Mat getTextMat(cv::Mat& mat,const std::string& desc1,int fontface,double scale,int thickness,const cv::Scalar& color)
{
    std::string desc=desc1;
    std::vector<std::string> tokens;
    boost::algorithm::replace_all(desc,"\t","    ");
    boost::algorithm::split(    tokens,
                                desc,
                                [&](auto ch)->bool{return ch=='\n';},
                                boost::algorithm::token_compress_on
                                );
    std::vector<cv::Point> points;
    points.reserve(tokens.size());
    size_t point_idx=0;
    cv::Size text_region_size=cv::Size{};
    int space=2;
    int xBase=2;
    for(auto& str:tokens)
    {
        int baseline=0;
        auto newSize= cv::getTextSize(str.c_str(),fontface,scale,thickness,&baseline);
        text_region_size=cv::Size(  std::max(text_region_size.width,newSize.width+xBase),
                                    std::max(text_region_size.height+space+newSize.height,newSize.height));
        //cv::Point point(xBase,text_region_size.height-baseline);
        points.emplace_back(xBase,text_region_size.height-baseline);
    }
    cv::Size matSize(std::max(mat.cols,text_region_size.width),std::max(mat.rows,text_region_size.height));
    cv::Mat* pMat=nullptr;
    cv::Mat newMat;
    if(matSize.width!=mat.cols || matSize.height!= mat.rows)
    {
        newMat=cv::Mat(matSize,CV_8UC3,{0,0,0});
        pMat=& newMat;
    }
    else
    {
        mat.setTo(cv::Scalar{0,0,0});
        pMat=&mat;
    }
    for(;point_idx<tokens.size();point_idx++)
    {
        const auto& p=points[point_idx];
        const auto& str=tokens[point_idx];
        cv::putText(*pMat,str.c_str(),p,fontface,scale,color,thickness,true);
    }
    return *pMat;
    //int baseline=0;
    //for(auto& str:{fmt::v7::format( "w {0}"               ,w              ),                                  
    //               fmt::v7::format( "loop {0}"            ,loop           ),                  
    //               fmt::v7::format( "buf_warped_size {0}" ,buf_warped_size),                              
    //               fmt::v7::format( "goodCount {0}"       ,goodCount      ),                      
    //               fmt::v7::format( "badCount {0}"        ,badCount       ),                      
    //               fmt::v7::format( "ratio {0}"           ,ratio          ) }                  
    //            )
    //{
    //    
    //}
    //return mat;
}

}

void debugBreakOnConditon(const std::string& msg,bool condition)
{
    if(!condition)
    {
        LOGF(WARNING,"condition \"%s\" doesn't met,breaking",msg.c_str());
#ifdef USE_SIGNAL
        raise(SIGTRAP);
#elif USE_WIN
        __debugbreak();
#endif
    }
}

