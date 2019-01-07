//
// Created by wang on 19-1-5.
//

#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace Simple_SLAM
{
    /** 图像的预处理过程Pre-process Input
     *
     * @param imRGB
     * @param imD
     * @param timestamp
     * @return
     */
    cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp) {
        mImGray = imRGB;
        cv::Mat imDepth = imD;

        cv::cvtColor()
        if(3 == mImGray.channels())
        {
            // 没什么卵用，就是看图像rgb的顺序
            cvtColor(mImGray, mImGray, mbRGB ? CV_RGB2GRAY : CV_BGR2GRAY);
        }
        else if(mImGray.channels()==4)
        {
            cvtColor(mImGray, mImGray, mbRGB ? CV_RGBA2GRAY : CV_BGRA2GRAY);
        }

        if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
            imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);
        /**
         * 接着初始化帧数据，也就是将图像数据转换为系统能够识别的Frame帧，计算图像的ORB特征，在这一步进行。
         */
        mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

        Track();

        return mCurrentFrame.mTcw.clone();
    }

}