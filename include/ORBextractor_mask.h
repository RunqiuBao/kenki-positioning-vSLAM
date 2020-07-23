/**
*
* Runqiu Bao (University of Tokyo)
* For more information see <https://github.com/RunqiuBao/kenki-posi>
*
*/
/**
* This file is part of ORB-SLAM2.
* This file is based on the file orb.cpp from the OpenCV library (see BSD license below).
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>
#include "MaskInfoSD.h"
#include "MaskInfo.h"
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <cuda/Fast.hpp>
#include <cuda/Orb.hpp>

namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractormask
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractormask(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractormask(){}

    static bool CompareSmall(std::pair<int,ExtractorNode*> &p1, std::pair<int,ExtractorNode*> &p2){
        return (p1.first<p2.first);
    }

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors, int leftrightflag, std::vector<cv::KeyPoint>& _keypoints_all, cv::OutputArray _descriptors_all, int extractTwice);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    bool mvImagePyramidAllocatedFlag;
    std::vector<cv::cuda::GpuMat>  mvImagePyramid;
    std::vector<cv::cuda::GpuMat>  mvImagePyramidBorder;

protected:
    //runqiu:mask for DO
    int maskFunction_bin(float x, float y, cv::Mat maskimage, cv::Mat maskimagePixel, const MaskSetSD &masksSDForThisFrame);

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints, int leftrightflag, cv::Mat maskimage, std::vector<std::vector<cv::KeyPoint> >& allKeypoints_all, int extractTwice);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;
    cv::Ptr<cv::cuda::Filter> mpGaussianFilter;
    cuda::Stream mcvStream;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;
    int kid;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    cuda::GpuFast gpuFast;
    cuda::IC_Angle ic_angle;
    cuda::IC_Angle ic_angle_all;
    cuda::GpuOrb gpuOrb;
};

} //namespace ORB_SLAM

#endif

