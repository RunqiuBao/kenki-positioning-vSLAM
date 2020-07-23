/**
*
* Runqiu Bao (University of Tokyo)
* For more information see <https://github.com/RunqiuBao/kenki-posi>
*
*/
#include<opencv2/core/core.hpp>

#ifndef MYSTRUCT_H2
#define MYSTRUCT_H2

struct OneStereoFrame{//runqiu: one stereo frame including pose and maskset
    bool masksDetected;
    string timestamp;
    cv::Mat leftImage;
    cv::Mat rightImage;
    vector<cv::KeyPoint> kpleft;
    vector<cv::KeyPoint> kpright;
    cv::Mat desleft;
    cv::Mat desright;
    cv::Mat projMatLeft;
    cv::Mat projMatRight;
    cv::Mat RTleftw;
    cv::Mat RTrightw;
    MaskSetSD masksInLeft;
};

#endif
