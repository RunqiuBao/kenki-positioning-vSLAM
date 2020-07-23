/**
*
* Runqiu Bao (University of Tokyo)
* For more information see <https://github.com/RunqiuBao/kenki-posi>
*
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include<System.h>
#include "MaskInfo.h"
#include "MaskInfoSD.h"

#include <sys/stat.h>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include "Converter.h"
#include "StereoFrameInfo.h"
#include <queue>//runqiu:使用queue来储存一秒以内的立体帧，包括相机位姿和mask的信息
//runqiu: time analysis
#include <chrono>


using namespace std;

vector<string> split(const string &s, char delim) {
    vector<string> elems;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
    if (!item.empty()) {
            elems.push_back(item);
        }
    }
    return elems;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps, vector<string> &vImgNames)
{
    ifstream fTimes;
    ifstream fPaths;
    string strPathFile = strPathToSequence + "mav0/timestamps.txt";
    string strPathTimeFile = strPathToSequence + "mav0/data.csv";
    fTimes.open(strPathTimeFile.c_str());
    fPaths.open(strPathFile.c_str());
    std::string temp;
    getline(fTimes,temp);

    while(!fPaths.eof())
    {
        string s;
        getline(fPaths,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            std::string t2;
            ss >> t2;
            vImgNames.push_back(t2);
        }

        s="";
        getline(fTimes,s);
        if(!s.empty()){
            stringstream ss;//runqiu: for separation by coma
            string temp2;
            ss << s;
            getline(ss,temp2, ',');
            getline(ss,temp2, ',');
            getline(ss,temp2, ',');
            getline(ss,temp2, ',');
            vTimestamps.push_back(std::stod(temp2));
        }
    }

    string strPrefixLeft = strPathToSequence + "mav0/cam0/data/";
    string strPrefixRight = strPathToSequence + "mav0/cam1/data/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        vstrImageLeft[i] = strPrefixLeft + to_string(i+1) + ".png";
        vstrImageRight[i] = strPrefixRight + to_string(i+1) + ".png";
    }
}

//runqiu: test if file exits
inline bool exists_test (const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }   
}

void LoadMasks(const string &strPathToMask, vector<string> &vstrMaskLeft, vector<string> &vstrMaskedFrame, vector<string> &vImgNames)
{
    string strPrefixMaskLeft = strPathToMask + "mav0/bbox-mask/";
    string strPrefixMaskedFrame = strPathToMask + "mav0/cam0/data/";
    string strPrefixMaskedFrame2 = strPathToMask + "mav0/cam0/data/";
    int nTimes = vImgNames.size();
    vstrMaskLeft.resize(nTimes);
    vstrMaskedFrame.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrMaskLeft[i] = strPrefixMaskLeft + to_string(i+1) + ".png";
        string maskedFrame=strPrefixMaskedFrame+to_string(i+1)+".png";
        if(exists_test(maskedFrame))
            vstrMaskedFrame[i]=maskedFrame;
        else
        {
            maskedFrame=strPrefixMaskedFrame2+to_string(i+1)+".png";//runqiu: miss some frames that don't contain objects
            vstrMaskedFrame[i]=maskedFrame;
        }
        

    }
}

void LoadMasksPixel(const string &strPathToMask, vector<string> &vstrMaskLeft, vector<string> &vImgNames)
{
    string strPrefixMaskLeft = strPathToMask + "mav0/pixel-wise-mask/";
    int nTimes = vImgNames.size();
    vstrMaskLeft.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrMaskLeft[i] = strPrefixMaskLeft + to_string(i+1) + ".png";
    }
}

void LoadMasksLabels(map<string, MaskSetSD> &masksSetDict, vector<string> masksText)
{
    int count=0;
    for (std::vector<string>::iterator it=masksText.begin(); it != masksText.end(); ++it)
    {
        vector<string> oneFrameMasks = split(*it, ' ');
        int obsNb=(oneFrameMasks.size()-1)/5;//objects number
        MaskSetSD newObSet;//runqiu: masks set for one frame
        for(int i=0;i<obsNb;i++){
            string newlabel=oneFrameMasks.at(1+i);
            newObSet.labels.push_back(newlabel);
            bbox newbbox;
	        try {
		        newbbox.lux=stoi(oneFrameMasks.at(1+obsNb+i*4));
	        }
	        catch (char *str) {
		        cout << str;
	        }
            
            newbbox.luy=stoi(oneFrameMasks.at(1+obsNb+i*4+1));
            newbbox.rlx=stoi(oneFrameMasks.at(1+obsNb+i*4+2));
            newbbox.rly=stoi(oneFrameMasks.at(1+obsNb+i*4+3));
            newObSet.bboxes.push_back(newbbox);
            newObSet.isdynamic.push_back(2);//runqiu:默认所有检出的物体都是not in ref_frame或者lost tracking of the object
        }
        string frameName=oneFrameMasks.at(0);
        masksSetDict[frameName]=newObSet;
	    count++;
    }
}

void saveStatistics(string resultPath, vector<cv::Mat> faketwcrecord, vector<cv::Mat> fakeRwcrecord, vector<double> vTimestampsrecord, vector<float> vTimesTrack, float medianTT, float meanTT, int maskrcnnCount){
    ofstream f;
    f.open(resultPath+"faketraj.txt");
    f << fixed;
    vector<cv::Mat>::iterator iter2 = faketwcrecord.begin();
    vector<double>::iterator iter3 = vTimestampsrecord.begin();
    for (vector<cv::Mat>::iterator iter = fakeRwcrecord.begin(); iter != fakeRwcrecord.end(); iter++, iter2++, iter3++)
	{
        vector<float> qwc = ORB_SLAM2::Converter::toQuaternion(*iter);
		f << setprecision(6) << std::to_string(*iter3) << setprecision(9) << " " << (*iter2).at<float>(0) << " " << (*iter2).at<float>(1) << " " << (*iter2).at<float>(2)
        << " " << qwc[0] << " " << qwc[1] << " " << qwc[2] << " " << qwc[3] << endl;
	}
    f.close();

    f.open(resultPath+"trackingTime.txt");
    f<<fixed;
    f<<"median tracking time: " << medianTT << endl;
    f<<"mean tracking time: " << meanTT << endl;
    f<<"maskrcnnCount: " << maskrcnnCount << endl;
    f.close();
}

//runqiu: argv是指向指针的指针，main函数的第二个参数“char *argv[]“也可以替换为 “char **argv“，两者是等价的。
int main(int argc, char **argv)
{
    if(argc != 10)
    {
        cerr << endl << "Usage: ./stereo_ral path_to_voca path_to_dataset settings semantictextlist result_path mask_threshold usingColorImage distance2refframe frameskip" << endl;
        return 1;
    }

    // variables definition
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    vector<string> vImgNames;
    vector<string> vstrMaskLeft;
    vector<string> vstrMaskPixelLeft;
    vector<string> vstrMaskedFrame;

    string path_to_voca(argv[1]);
    string path_to_dataset(argv[2]);
    string setting(argv[3]);
    string maskTextlist(argv[4]);//runqiu: semantics meaning of the masks
    string resultPath(argv[5]);//runqiu: folder to save the result
    float maskThr;
    maskThr=atof(argv[6]);
    int usingColorImage, refFrameDist, frameskip;
    usingColorImage=atoi(argv[7]);//runqiu:输入的图片是color图吗
    refFrameDist=atoi(argv[8]);
    frameskip=atoi(argv[9]);
    cout<<"maskThr: "<<maskThr<<endl;
    cout<<"frameskip: "<<frameskip<<endl;
    cout<<"refFrameDist: "<<refFrameDist<<endl;
    cout<<"usingColorImage: "<<usingColorImage<<endl;

    // load images 
    LoadImages(path_to_dataset, vstrImageLeft, vstrImageRight, vTimestamps, vImgNames);//runqiu: vTimestamps is double numbers, vImgNames is exactly storing the image names
    LoadMasks(path_to_dataset, vstrMaskLeft, vstrMaskedFrame, vImgNames);
    LoadMasksPixel(path_to_dataset, vstrMaskPixelLeft, vImgNames);
    const int nImages = vstrImageLeft.size();
    
    // runqiu: read masks and semantic labels for every frame
    ifstream inFile(path_to_dataset+maskTextlist);
    vector<string> masksText;
    if(!inFile)
    {
	cout<<"Couldn't open the semantic text file"<<endl;
	exit(1);
    }
    string line;
    while( getline(inFile, line)  )
    {
        masksText.push_back(line);
    }
    map<string, MaskSetSD> masksSetDict;
    LoadMasksLabels(masksSetDict, masksText);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    cout<<path_to_dataset+setting<<endl;
    cout<<path_to_voca<<endl;
    ORB_SLAM2::System SLAM(path_to_voca,path_to_dataset+setting,ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    // vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop    
    cv::Mat imLeft, imRight;
    cv::Mat imMask;
    cv::Mat imMaskPixel;
    cv::Mat imMaskYolo;
    vector<cv::Mat> fakeRwcrecord;//runqiu: 用来储存tracking结果的相机位姿
    vector<cv::Mat> faketwcrecord;
    vector<double> vTimestampsrecord;//runqiu: 用来储存实际处理的帧的时间戳
    //runqiu:用来储存动体判断的队列
    queue<OneStereoFrame> stereoFramesQueue;

    int maskrcnnCount=0;
    for(int ni=0; ni<nImages; ni=ni+frameskip)//nImages
    {
        cout<<to_string(ni)<<endl;
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        // runqiu: read mask image from file
        imMask = cv::imread(vstrMaskLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imMaskPixel = cv::imread(vstrMaskPixelLeft[ni], CV_LOAD_IMAGE_UNCHANGED);
        imMaskYolo = cv::imread(vstrMaskedFrame[ni], CV_LOAD_IMAGE_UNCHANGED);//runqiu: 用yolo的标记结果可视化
        MaskSetSD masksForThisFrame;
        masksForThisFrame=masksSetDict[to_string(ni)];
        double tframe = vTimestamps[ni];
        SLAM.mpTracker->mImGrayYOLO = imLeft.clone();

        // runqiu: 计算mask面积
        int grayImgNum = 1;                             
	    int grayChannels = 0 ;                          
	    cv::Mat grayHist;                           
	    const int grayHistDim = 1;              
	    const int grayHistSize = 256 ;                     
	    float grayRanges[2] = { 0, 256 };                 
	    const float *grayHistRanges[1] = { grayRanges };   
	    bool grayUniform = true;                           
	    bool grayAccumulate = false; 
        // 计算灰度图像的直方图
	    cv::calcHist( &imMaskPixel, 
                  grayImgNum, 
                  &grayChannels, 
                  cv::Mat(), 
                  grayHist, 
                  grayHistDim, 
                  &grayHistSize, 
                  grayHistRanges, 
                  grayUniform, 
                  grayAccumulate );
	    float white_val = grayHist.at<float>(255);
        float black_val = grayHist.at<float>(0);
        float doi=white_val/(white_val+black_val);
        cout<<"MAR:"<<to_string(doi)<<endl;
        if(doi>maskThr){//runqiu: set the threshold for switching to mask-rcnn
            imMask=imMaskPixel;//runqiu: hierarchical mask generation
            cout<<"Mask-RCNN:"<<vImgNames[ni]<<endl;
	        maskrcnnCount++;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        chrono::steady_clock sc;   // create an object of `steady_clock` class for detailed time statistics

        // Pass the images to the SLAM system
        cv::Mat Tcw;
        auto start = sc.now();     // start timer
        if(stereoFramesQueue.size()==refFrameDist && masksForThisFrame.labels.size()!=0){
            Tcw = SLAM.TrackStereo_nu(imLeft,imRight,tframe,imMask,masksForThisFrame);//只用一张mask图解决所有问题。masksSDSet用于指导显示文字
        }
        else
        {
            Tcw = SLAM.TrackStereo(imLeft,imRight,tframe,imMask,masksForThisFrame);//只用一张mask图解决所有问题。masksSDSet用于指导显示文字
        }
        auto end = sc.now();       // end timer (starting & ending is done by measuring the time at the moment the process started & ended respectively)
        auto time_span = static_cast<chrono::duration<double>>(end - start);   // measure time span between start & end
        cout<<"1stTrack: "<<time_span.count()<<" seconds !!!"<<endl;
        
        //runqiu: 先将当前帧的内容压进队列，然后取出一秒之前的帧和当前帧进行动体判断的计算,然后根据计算结果设置当前帧mask的运动状态
        OneStereoFrame curr_frame;
        curr_frame.masksDetected=(masksForThisFrame.labels.size()!=0) ? false : true;//runqiu:当前帧的mask为空，没有东西被检测到
        curr_frame.timestamp=vImgNames[ni];
        cv::Mat imLeftgray, imRightgray;
	    if(usingColorImage==1){//runqiu: 把图像变成灰度图
		    cv::cvtColor(imLeft, imLeftgray, CV_RGB2GRAY);
        	cv::cvtColor(imRight, imRightgray, CV_RGB2GRAY);
        	curr_frame.leftImage=imLeftgray;
        	curr_frame.rightImage=imRightgray;        
	    }   
        else{
	    	curr_frame.leftImage=imLeft;
        	curr_frame.rightImage=imRight;
	    }
        //runqiu: prepare stereo frame paras
        cv::Mat intri = SLAM.mpTracker->mK;
        cv::Mat_<float> RTleftwc, Rcw(3,3), tcw(3,1), Rwc(3,3), twc(3,1), projMatLeftwc, Twc;
        Rcw=Tcw.rowRange(0,3).colRange(0,3);
        tcw=Tcw.rowRange(0,3).col(3);
        Rwc=Rcw.t();
        twc=-Rwc*tcw;
        cv::hconcat(Rwc, twc, RTleftwc);//runqiu: horizontally concatenate two opencv matrix
        projMatLeftwc=intri*RTleftwc;
        cv::Mat_<float> Hb(1,4);
        Hb<< 0, 0, 0, 1;
        cv::vconcat(RTleftwc,Hb,Twc);
        cv::Mat Hlr = cv::Mat::eye(4,4,CV_32F);//runqiu:左右目之间的变换矩阵
        float baseline = -1*(SLAM.mpTracker->mbf)/(intri.at<float>(0,0));
        Hlr.at<float>(0,3) = baseline;
        cv::Mat projMatRightwc, Twcright;
        Twcright=Hlr.inv()*Twc;
        cv::Mat RTrightwc;
        RTrightwc=Twcright.rowRange(0,3);
        projMatRightwc=intri*RTrightwc;
        cv::Mat RTleftcw, RTrightcw;
        cv::hconcat(Rcw, tcw, RTleftcw);
        cv::Mat_<float> Hlrw(4,4);
        cv::Mat Trightcw;
        Trightcw=Tcw*Hlr;
        RTrightcw=Trightcw.rowRange(0,3);
        //runqiu: write into curr_frame
        curr_frame.projMatLeft=projMatLeftwc;
        curr_frame.projMatRight=projMatRightwc;
        curr_frame.RTleftw=RTleftcw;
        curr_frame.RTrightw=RTrightcw;
        curr_frame.masksInLeft=masksForThisFrame;
        curr_frame.kpleft=SLAM.mpTracker->mCurrentFrame.mvKeys_all;
        curr_frame.kpright=SLAM.mpTracker->mCurrentFrame.mvKeysRight;
        curr_frame.desleft=SLAM.mpTracker->mCurrentFrame.mDescriptors_all;
        curr_frame.desright=SLAM.mpTracker->mCurrentFrame.mDescriptorsRight;
        
        if(stereoFramesQueue.size()==refFrameDist){//runqiu: queue中积累了足够的帧之后就开始动体判断
            OneStereoFrame ref_frame=stereoFramesQueue.front();
            stereoFramesQueue.pop();//runqiu:pop之后该元素就被删除掉了
            stereoFramesQueue.push(curr_frame);
            //runqiu:利用ref_frame和curr_frame进行当前帧的动体判断
            start = sc.now();     // start timer
            if(masksForThisFrame.labels.size()!=0){
                SLAM.DistinguishDO(ref_frame, curr_frame, imMask, masksForThisFrame);
                end = sc.now();       // end timer (starting & ending is done by measuring the time at the moment the process started & ended respectively)
                time_span = static_cast<chrono::duration<double>>(end - start);   // measure time span between start & end
                cout<<"SD: "<<time_span.count()<<" seconds !!!"<<endl;
                start = sc.now();
                SLAM.mpTracker->extractTwice = 1;
                SLAM.TrackStereo(imLeft,imRight, tframe, imMask, masksForThisFrame);//runqiu: track twice
                SLAM.mpTracker->extractTwice = 0;
            }
            end = sc.now();       // end timer (starting & ending is done by measuring the time at the moment the process started & ended respectively)
            time_span = static_cast<chrono::duration<double>>(end - start);   // measure time span between start & end
            cout<<"2nd tracking: "<<time_span.count()<<" seconds !!!"<<endl;
        }
        else{
            stereoFramesQueue.push(curr_frame);
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        //runqiu: 记录tracking后获得的相机位姿results
        fakeRwcrecord.push_back(Rwc);//runqiu：记录了直接tracking的结果
        faketwcrecord.push_back(twc);//runqiu：记录了直接tracking的结果
        vTimestampsrecord.push_back(tframe);//runqiu：记录了当前帧的时间戳

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack.push_back(ttrack);//runqiu：记录了本次tracking的总耗时
            
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    float actualNImages = vTimesTrack.size();
    for(int ni=0; ni<actualNImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    float medianTT=vTimesTrack[actualNImages/2];
    float meanTT=totaltime*frameskip/nImages;
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << medianTT << endl;
    cout << "mean tracking time: " << meanTT << endl;

    //save statistics
    saveStatistics(path_to_dataset+resultPath, faketwcrecord, fakeRwcrecord, vTimestampsrecord, vTimesTrack, medianTT, meanTT, maskrcnnCount);

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryAndMap(SLAM.GetMap(), path_to_dataset+resultPath+"KeyFrameTrajectory.txt", path_to_dataset+resultPath+"MapPoints.txt");//runqiu:to save the point cloud map with trajectory
    SLAM.SaveTrajectoryKITTI(path_to_dataset+resultPath+"CameraTrajectory.txt");

    return 0;
    
}
