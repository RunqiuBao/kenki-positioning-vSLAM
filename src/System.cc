/**
*
* Runqiu Bao (University of Tokyo)
* For more information see <https://github.com/RunqiuBao/kenki-posi>
*
*/
/**
* This file is part of ORB-SLAM2.
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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "MaskInfo.h"
#include "MaskInfoSD.h"
#include "StereoFrameInfo.h"
#include <opencv2/opencv.hpp>
#include"Frame.h"
//runqiu: time analysis
#include <chrono>
#include "histogram.h"

//runqiu:bin-voca
bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)//runqiu: initialization list
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    clock_t tStart = clock();//runqiu:bin-voca

    mpVocabulary = new ORBVocabulary();
    //bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);

    //runqiu:bin-voca
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
	  bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
	else
	  bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);

    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        //cerr << "Falied to open at: " << strVocFile << endl;
	cerr << "Failed to open at: " << strVocFile << endl;//runqiu:bin-voca
        exit(-1);
    }
    //cout << "Vocabulary loaded!" << endl << endl;
    printf("Vocabulary loaded in %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);//runqiu:bin-voca

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    /* runqiu:暂时关闭显示*/
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const cv::Mat &imMask, const MaskSetSD masksForThisFrame)
{

    mpTracker->masksSDThisFrame = masksForThisFrame;
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp,imMask);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackStereo_nu(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const cv::Mat &imMask, const MaskSetSD masksForThisFrame)
{
    mpTracker->masksSDThisFrame = masksForThisFrame;
    cv::Mat Tcw = mpTracker->GrabImageStereo_nu(imLeft,imRight,timestamp,imMask);

    return Tcw;
}


cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

//runqiu:to save the point cloud map with trajectory
void System::SaveKeyFrameTrajectoryAndMap(ORB_SLAM2::Map *map, const string &filename, const string &tracksfile) {
    std::cout << std::endl << "Saving keyframe trajectory to " << filename << " ..." << std::endl;

    vector<ORB_SLAM2::KeyFrame*> vpKFs = map->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

    std::ofstream f;
    f.open(filename.c_str());
    f << fixed;

    std::ofstream fpoints;
    fpoints.open(tracksfile.c_str());
    fpoints << fixed;

    for(size_t i = 0; i < vpKFs.size(); i++) {
        ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3]<< " " <<std::to_string(pKF->N)<< std::endl;//runqiu: save trajectory

        for (auto point : pKF->GetMapPoints()) {
            auto coords = point->GetWorldPos();
            fpoints << setprecision(6)
                    << pKF->mTimeStamp
                    << " " << point->mnId
                    << setprecision(7)
                    << " " << coords.at<float>(0, 0)
                    << " " << coords.at<float>(1, 0)
                    << " " << coords.at<float>(2, 0)
                    << std::endl;//runqiu: save mappoints
        }
    }

    f.close();//runqiu: trajectory
    fpoints.close();//runqiu: mappoints
    std::cout << std::endl << "trajectory saved!" << std::endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;//runqiu: relative framepose times reference framepose equals arbitrary framepose
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);//runqiu: inverse of homogeneous tf
        vector<float> qwc = Converter::toQuaternion(Rwc);
        f << setprecision(6) << *lT << setprecision(9) << " " << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2)
          << " " << qwc[0] << " " << qwc[1] << " " << qwc[2] << " " << qwc[3] << endl;

        //f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
        //     Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
        //     Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

//runqiu:判断vector的某一元素是否存在,不存在则返回-1，存在则返回index
int System::is_element_in_vector(vector<int> v,int element){
	vector<int>::iterator it;
	it=find(v.begin(),v.end(),element);
	if (it!=v.end()){
		return std::distance(v.begin(), it);
	}
	else{
		return -1;
	}
}

void System::DistinguishDO(OneStereoFrame &ref_frame, OneStereoFrame &curr_frame, cv::Mat &imMask, MaskSetSD &masksSDForThisFrame){
    //runqiu:如果当前帧不含mask，不会进入本函数
    chrono::steady_clock sc;   // create an object of `steady_clock` class
    auto start = sc.now();     // start timer
    //#runqiu:先处理referance frame#
    std::vector<cv::KeyPoint> kpleft;
    cv::Mat desleft;
    std::vector<cv::KeyPoint> kpright;
    cv::Mat desright;
    kpleft=ref_frame.kpleft;
    kpright=ref_frame.kpright;
    desleft=ref_frame.desleft;
    desright=ref_frame.desright;


    //物体识别
    MaskSetSD masksThisFrame=ref_frame.masksInLeft;
    
    //物体level的特征点集合
    vector<vector<cv::KeyPoint>> objectsKpLeft;
    vector<cv::Mat> objectsDesLeft;
    vector<vector<int>> objectsKpLeft_index;
    int maskNum=masksThisFrame.labels.size();//runqiu:该帧所含的mask数量
    objectsKpLeft.resize(maskNum+1);//runqiu:最后一个物体是静态环境
    objectsDesLeft.resize(maskNum+1);
    objectsKpLeft_index.resize(maskNum+1);
    for(int i=0;i<kpleft.size();i++){
        cv::KeyPoint kpoint=kpleft[i];
        bool enviPoint=true;
        for(int j=0;j<maskNum;j++){
            if(kpoint.pt.x>masksThisFrame.bboxes[j].lux && kpoint.pt.x<masksThisFrame.bboxes[j].rlx && kpoint.pt.y>masksThisFrame.bboxes[j].luy && kpoint.pt.y<masksThisFrame.bboxes[j].rly){
                bool takeit=true;
                for(int k=0;k<maskNum;k++){
                    if(k!=j){
                        if(kpoint.pt.x>masksThisFrame.bboxes[k].lux && kpoint.pt.x<masksThisFrame.bboxes[k].rlx && kpoint.pt.y>masksThisFrame.bboxes[k].luy && kpoint.pt.y<masksThisFrame.bboxes[k].rly){
                            takeit=false;
                        }   
                    }
                }
                if(takeit){
                    objectsKpLeft[j].push_back(kpoint);
                    cv::Mat oldDes = objectsDesLeft[j];
                    if(oldDes.empty())
                        objectsDesLeft[j] = desleft.rowRange(i,i+1);
                    else
                        cv::vconcat(oldDes,desleft.rowRange(i,i+1),objectsDesLeft[j]);
                    objectsKpLeft_index[j].push_back(i);
                    enviPoint=false;
                }
                break;
            }
        }
        if(enviPoint){
            objectsKpLeft[maskNum].push_back(kpoint);
            cv::Mat oldDes = objectsDesLeft[maskNum];
            if(oldDes.empty())
                objectsDesLeft[maskNum] = desleft.rowRange(i,i+1);
            else
            {
                cv::vconcat(oldDes,desleft.rowRange(i,i+1),objectsDesLeft[maskNum]);
            }
            objectsKpLeft_index[maskNum].push_back(i);
        }
        
    }

    //runqiu:在右图中match各物体特征点，用knn match提高match速度，记录匹配到的特征点的index
    vector<vector<int>> matchedKps_indexinKpLeft;
    vector<vector<int>> matchedKps_indexinKpRight;
    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1));
    matchedKps_indexinKpLeft.resize(objectsKpLeft.size());
    matchedKps_indexinKpRight.resize(objectsKpLeft.size());
    for(int i=0;i<objectsKpLeft.size();i++){
        std::vector<std::vector<cv::DMatch>> matches;//runqiu:每个点有两个候补匹配结果，所以构成一个嵌套vector
        std::vector<cv::DMatch> oneObjmatch;
        matcher.knnMatch(objectsDesLeft[i],desright,matches,2);
        if(matches.empty()){
            continue;
        }
        for(int j=0;j<matches.size();j++){
            if(matches[j][0].distance<0.7*matches[j][1].distance){
                matchedKps_indexinKpLeft[i].push_back(objectsKpLeft_index[i][matches[j][0].queryIdx]);
                matchedKps_indexinKpRight[i].push_back(matches[j][0].trainIdx);
                oneObjmatch.push_back(matches[j][0]);
            }
        }
        //-- Draw matches
        /* 
        cv::Mat img_matches;
        cv::drawMatches( imageOneleft, objectsKpLeft[i], imageOneright, kpright, oneObjmatch, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        cv::imshow("Good Matches", img_matches );
        cv::waitKey();
        */
        
    }

    //runqiu:三角测量
    vector<cv::Mat> DOs_origin;
    vector<vector<cv::Point3f>> DOs_origin_np;//runqiu: cv2的三角测量函数返回的是归一化坐标（四维），需要后处理成3d坐标
    cv::Mat projMatLeft = ref_frame.projMatLeft;
    cv::Mat projMatRight = ref_frame.projMatRight;
    DOs_origin.resize(matchedKps_indexinKpLeft.size());
    DOs_origin_np.resize(DOs_origin.size());
    for(int i=0;i<matchedKps_indexinKpLeft.size();i++){
        //cv::Mat kpsetleft(matchedKps_indexinKpLeft[i].size(), 1, CV_64FC(2));
        vector<cv::Point2f> kpsetleft;
        for(int j=0;j<matchedKps_indexinKpLeft[i].size();j++){
            //kpsetleft.at<cv::Vec3f>(j,0)[0] = kpleft[matchedKps_indexinKpLeft[i][j]].pt.x;
            //kpsetleft.at<cv::Vec3f>(j,0)[1] = kpleft[matchedKps_indexinKpLeft[i][j]].pt.y;
            cv::Point2f onePoint;
            onePoint.x = kpleft[matchedKps_indexinKpLeft[i][j]].pt.x;
            onePoint.y = kpleft[matchedKps_indexinKpLeft[i][j]].pt.y;
            kpsetleft.push_back(onePoint);
        }
        //cv::Mat kpsetright(matchedKps_indexinKpRight[i].size(), 1, CV_64FC(2));
        vector<cv::Point2f> kpsetright;
        for(int j=0;j<matchedKps_indexinKpRight[i].size();j++){
            //kpsetright.at<cv::Vec3f>(j,0)[0] = kpright[matchedKps_indexinKpRight[i][j]].pt.x;
            //kpsetright.at<cv::Vec3f>(j,0)[1] = kpright[matchedKps_indexinKpRight[i][j]].pt.y;
            cv::Point2f onePoint;
            onePoint.x = kpright[matchedKps_indexinKpRight[i][j]].pt.x;
            onePoint.y = kpright[matchedKps_indexinKpRight[i][j]].pt.y;
            kpsetright.push_back(onePoint);
        }
        if(kpsetleft.empty())
            continue;
        else{
            cv::triangulatePoints(projMatLeft,projMatRight,kpsetleft,kpsetright,DOs_origin[i]);//runqiu:结果点的坐标是相机坐标
            for(int j=0;j<DOs_origin[i].cols;j++){
                cv::Mat onePoint(1,4,cv::DataType<float>::type);
                onePoint.at<float>(0,0) =DOs_origin[i].at<float>(0,j);
                onePoint.at<float>(0,1) =DOs_origin[i].at<float>(1,j);
                onePoint.at<float>(0,2) =DOs_origin[i].at<float>(2,j);
                onePoint.at<float>(0,3) =DOs_origin[i].at<float>(3,j);
                vector<cv::Point3f> one3dPoint;
                cv::convertPointsFromHomogeneous(onePoint, one3dPoint);//runqiu: 这个函数很奇怪，onepoint只能是单个点，one3dpoint却需要是一个vector去接
                one3dPoint[0].x=one3dPoint[0].x;//runqiu:测出来的3d点的坐标和好像和相机自身坐标差一个负号
                one3dPoint[0].y=one3dPoint[0].y;
                one3dPoint[0].z=one3dPoint[0].z;
                DOs_origin_np[i].push_back(one3dPoint[0]);
            }
            
        }
            
    }
    //runqiu:至此参考帧处理完了
    
    //runqiu:开始处理当前帧
    std::vector<cv::KeyPoint> kpleft_next;
    cv::Mat desleft_next;
    std::vector<cv::KeyPoint> kpright_next;
    cv::Mat desright_next;
    kpleft_next=curr_frame.kpleft;
    kpright_next=curr_frame.kpright;
    desleft_next=curr_frame.desleft;
    desright_next=curr_frame.desright;

    //runqiu:左图与上一帧左图match
    vector<vector<int>> matchedKps_indexinKpLeft_withnext;
    vector<vector<int>> matchedKps_indexinNextKpLeft_withlast;
    matchedKps_indexinKpLeft_withnext.resize(objectsKpLeft.size());
    matchedKps_indexinNextKpLeft_withlast.resize(objectsKpLeft.size());
    for(int i=0;i<objectsKpLeft.size();i++){
        std::vector<std::vector<cv::DMatch>> matches; 
        std::vector<cv::DMatch> oneObjmatch;
        matcher.knnMatch(objectsDesLeft[i],desleft_next,matches,2);
        if(matches.empty()){
            continue;
        }
        for(int j=0;j<matches.size();j++){
            if(matches[j][0].distance<0.7*matches[j][1].distance){
                matchedKps_indexinKpLeft_withnext[i].push_back(objectsKpLeft_index[i][matches[j][0].queryIdx]);
                matchedKps_indexinNextKpLeft_withlast[i].push_back(matches[j][0].trainIdx);
                oneObjmatch.push_back(matches[j][0]);
            }
        }
        /*
        if(!oneObjmatch.empty()){
            //-- Draw matches
            cv::Mat img_matches;
            vector<char> matchesMask;
            for(int i=0; i<oneObjmatch.size();i++){
                if((i%1)==0)
                    matchesMask.push_back(1);
                else
                {
                    matchesMask.push_back(0);
                }
                
            }
            cv::drawMatches( ref_frame.leftImage, objectsKpLeft[i], curr_frame.leftImage, kpleft_next, oneObjmatch, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), matchesMask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );//std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            //-- Show detected matches
            cv::imshow("Good Matches", img_matches );
            cv::waitKey(0);
        }*/
        
    }

    //runqiu:与当前帧右图match
    vector<vector<int>> matchedKps_indexinNextKpLeft;//记录的都是在该image所有特征点集合中的index,这样会方便查找
    vector<vector<int>> matchedKps_indexinNextKpRight;
    matchedKps_indexinNextKpLeft.resize(matchedKps_indexinNextKpLeft_withlast.size());
    matchedKps_indexinNextKpRight.resize(matchedKps_indexinNextKpLeft_withlast.size());
    for(int i=0;i<matchedKps_indexinNextKpLeft_withlast.size();i++){
        cv::Mat objkpinnextleft;//runqiu:记录kp描述子的临时变量
        vector<cv::KeyPoint> KPsinnextleft;
        for(int j=0;j<matchedKps_indexinNextKpLeft_withlast[i].size();j++){
            cv::Mat oldDes = objkpinnextleft;
            if(oldDes.empty()){
                objkpinnextleft = desleft_next.rowRange(matchedKps_indexinNextKpLeft_withlast[i][j], matchedKps_indexinNextKpLeft_withlast[i][j]+1);
            }
            else{
                cv::vconcat(oldDes,desleft_next.rowRange(matchedKps_indexinNextKpLeft_withlast[i][j], matchedKps_indexinNextKpLeft_withlast[i][j]+1), objkpinnextleft);
            }
            KPsinnextleft.push_back(kpleft_next[matchedKps_indexinNextKpLeft_withlast[i][j]]);
            //objkpinnextleft_kp.push_back(kpleft_next[matchedKps_indexinNextKpLeft_withlast[i][j]]);
        }
        vector<std::vector<cv::DMatch>> matches; 
        std::vector<cv::DMatch> oneObjmatch;
        matcher.knnMatch(objkpinnextleft,desright_next,matches,2);
        if(matches.empty()){
            continue;
        }
        for(int j=0;j<matches.size();j++){
            if(matches[j][0].distance<0.7*matches[j][1].distance){
                matchedKps_indexinNextKpLeft[i].push_back(matchedKps_indexinNextKpLeft_withlast[i][matches[j][0].queryIdx]);
                matchedKps_indexinNextKpRight[i].push_back(matches[j][0].trainIdx);
                oneObjmatch.push_back(matches[j][0]);
            }
        }
        /*
        if(!oneObjmatch.empty()){
            //-- Draw matches
            cv::Mat img_matches;
            cv::drawMatches( curr_frame.leftImage, KPsinnextleft, curr_frame.rightImage, kpright_next, oneObjmatch, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            //-- Show detected matches
            cv::imshow("Good Matches", img_matches );
            cv::waitKey();
        }*/

    }

    //runqiu:三角测量当前帧中各物体匹配上的特征点
    vector<cv::Mat> DOs_next;
    vector<vector<cv::Point3f>> DOs_next_np;//runqiu: cv2的三角测量函数返回的是归一化坐标（四维），需要后处理成3d坐标
    projMatLeft = curr_frame.projMatLeft;
    projMatRight = curr_frame.projMatRight;
    DOs_next.resize(matchedKps_indexinNextKpLeft.size());
    DOs_next_np.resize(DOs_next.size());
    for(int i=0;i<matchedKps_indexinNextKpLeft.size();i++){
        //cv::Mat kpsetleft(matchedKps_indexinNextKpLeft[i].size(), 1, CV_64FC(2));
        vector<cv::Point2f> kpsetleft;
        for(int j=0;j<matchedKps_indexinNextKpLeft[i].size();j++){
            //kpsetleft.at<cv::Vec3f>(j,0)[0] = kpleft_next[matchedKps_indexinNextKpLeft[i][j]].pt.x;
            //kpsetleft.at<cv::Vec3f>(j,0)[1] = kpleft_next[matchedKps_indexinNextKpLeft[i][j]].pt.y;
            cv::Point2f onePoint;
            onePoint.x = kpleft_next[matchedKps_indexinNextKpLeft[i][j]].pt.x;
            onePoint.y = kpleft_next[matchedKps_indexinNextKpLeft[i][j]].pt.y;
            kpsetleft.push_back(onePoint);
        }
        //cv::Mat kpsetright(matchedKps_indexinNextKpRight[i].size(), 1, CV_64FC(2));
        vector<cv::Point2f> kpsetright;
        for(int j=0;j<matchedKps_indexinNextKpRight[i].size();j++){
            //kpsetright.at<cv::Vec3f>(j,0)[0] = kpright_next[matchedKps_indexinNextKpRight[i][j]].pt.x;
            //kpsetright.at<cv::Vec3f>(j,0)[1] = kpright_next[matchedKps_indexinNextKpRight[i][j]].pt.y;
            cv::Point2f onePoint;
            onePoint.x = kpright_next[matchedKps_indexinNextKpRight[i][j]].pt.x;//runqiu:根据高翔的书
            onePoint.y = kpright_next[matchedKps_indexinNextKpRight[i][j]].pt.y;
            kpsetright.push_back(onePoint);
        }
        if(kpsetleft.empty())
            continue;
        else{
            cv::triangulatePoints(projMatLeft,projMatRight,kpsetleft,kpsetright,DOs_next[i]);
            for(int j=0;j<DOs_next[i].cols;j++){
                cv::Mat onePoint(1,4,cv::DataType<float>::type);
                onePoint.at<float>(0,0) =DOs_next[i].at<float>(0,j);
                onePoint.at<float>(0,1) =DOs_next[i].at<float>(1,j);
                onePoint.at<float>(0,2) =DOs_next[i].at<float>(2,j);
                onePoint.at<float>(0,3) =DOs_next[i].at<float>(3,j);
                vector<cv::Point3f> one3dPoint;
                cv::convertPointsFromHomogeneous(onePoint, one3dPoint);
                one3dPoint[0].x=one3dPoint[0].x;//runqiu:测出来的3d点的坐标和好像和相机自身坐标差一个负号
                one3dPoint[0].y=one3dPoint[0].y;
                one3dPoint[0].z=one3dPoint[0].z;
                //float tempx=-one3dPoint[0].z;
                //float tempy=-one3dPoint[0].y;
                //float tempz=one3dPoint[0].x;//runqiu:for kitti 05
                //one3dPoint[0].x=tempx;
                //one3dPoint[0].y=tempy;
                //one3dPoint[0].z=tempz;
                DOs_next_np[i].push_back(one3dPoint[0]);
            }
            
        }
            
    }

    float dynaThreshold=mpTracker->dynaThreshold;
    float meaRange=mpTracker->meaRange;
    float bkgMeaRange=mpTracker->bkgMeaRange;//runqiu:背景中的测量点太远的话，误差会较大，可能导致误判断静态物体
    float baseline=mpTracker->baseline;
    float stdev=mpTracker->stdev;

    //runqiu:找到各物体中点在参考帧和当前帧的中位数位置，并记录该中位数点在左图中的index
    vector<vector<cv::Point3f>> objPts_origin_record;//runqiu:最后在两帧间获得匹配的物体级特征点
    vector<vector<cv::Point3f>> objPts_next_record;
    vector<vector<int>> objPts_origin_record_indexinleft;
    vector<vector<int>> objPts_next_record_indexinleft;
    objPts_origin_record.resize(matchedKps_indexinNextKpLeft.size());
    objPts_next_record.resize(matchedKps_indexinNextKpLeft.size());
    objPts_origin_record_indexinleft.resize(matchedKps_indexinNextKpLeft.size());
    objPts_next_record_indexinleft.resize(matchedKps_indexinNextKpLeft.size());
    for(int i=0;i<matchedKps_indexinNextKpLeft.size();i++){
        if(matchedKps_indexinNextKpLeft[i].empty()){
            continue;
        }
        for(int j=0;j<matchedKps_indexinNextKpLeft[i].size();j++){
            int theindex = matchedKps_indexinNextKpLeft[i][j];
            //runqiu:这一步搜索一定能找到，因为 matchedKps_indexinNextKpLeft是matchedKps_indexinNextKpLeft_withlast的子集
            std::vector<int>::iterator it = std::find(matchedKps_indexinNextKpLeft_withlast[i].begin(), matchedKps_indexinNextKpLeft_withlast[i].end(), theindex);
            int theindex2 = std::distance(matchedKps_indexinNextKpLeft_withlast[i].begin(), it);
            int theindex_inlastleft = matchedKps_indexinKpLeft_withnext[i][theindex2];
            int theindex3 = is_element_in_vector(matchedKps_indexinKpLeft[i],theindex_inlastleft);
            if(theindex3!=-1){
                float distance1 = std::sqrt(std::pow((DOs_origin_np[i][theindex3].x-ref_frame.RTleftw.at<float>(0,3)),2)+std::pow((DOs_origin_np[i][theindex3].y-ref_frame.RTleftw.at<float>(1,3)),2)+std::pow((DOs_origin_np[i][theindex3].z-ref_frame.RTleftw.at<float>(2,3)),2));
                float distance2 = std::sqrt(std::pow((DOs_next_np[i][j].x-curr_frame.RTleftw.at<float>(0,3)),2)+std::pow((DOs_next_np[i][j].y-curr_frame.RTleftw.at<float>(1,3)),2)+std::pow((DOs_next_np[i][j].z-curr_frame.RTleftw.at<float>(2,3)),2));
                // float meaRange=40;
                // float bkgMeaRange=20;//runqiu:背景中的测量点太远的话，误差会较大，可能导致误判断静态物体
                // float baseline=1.064;
                if(i==2)
                    int a=1;
                if(i!=matchedKps_indexinNextKpLeft.size()-1){
                    if(distance1<=meaRange*baseline && distance2<=meaRange*baseline){//runqiu:远点不能用于translation的参考，因为远点可能是天空也可能是mismatching
                        objPts_origin_record[i].push_back(DOs_origin_np[i][theindex3]);
                        objPts_origin_record_indexinleft[i].push_back(theindex_inlastleft);
                        //std::vector<int>::iterator it = std::find(matchedKps_indexinNextKpLeft[i].begin(), matchedKps_indexinNextKpLeft[i].end(), theindex);
                        //int theindex4 = std::distance(matchedKps_indexinNextKpLeft[i].begin(), it);
                        objPts_next_record[i].push_back(DOs_next_np[i][j]);
                        objPts_next_record_indexinleft[i].push_back(theindex);
                    }
                }
                else{
                    if(distance1<=bkgMeaRange*baseline && distance2<=bkgMeaRange*baseline){//runqiu:远点不能用于translation的参考，因为远点可能是天空也可能是mismatching
                        objPts_origin_record[i].push_back(DOs_origin_np[i][theindex3]);
                        objPts_origin_record_indexinleft[i].push_back(theindex_inlastleft);
                        //std::vector<int>::iterator it = std::find(matchedKps_indexinNextKpLeft[i].begin(), matchedKps_indexinNextKpLeft[i].end(), theindex);
                        //int theindex4 = std::distance(matchedKps_indexinNextKpLeft[i].begin(), it);
                        objPts_next_record[i].push_back(DOs_next_np[i][j]);
                        objPts_next_record_indexinleft[i].push_back(theindex);
                    }
                }
            }
        }
        // cout<<"object "<<to_string(i)<<" points number before 10m limit: "<<to_string(objPts_origin_record[i].size())<<endl;
    }
    //runqiu:计算各物体移动中位数，并找到中位数对应的物体点
    vector<vector<float>> movederror_objptslist_list;
    vector<int> movederror_median_indexinnextleft;//runqiu:由于1013行进行了10m的限制，已经不能准确找到中位数点，但是不影响bbox的选择
    vector<float> movederror_median;
    vector<float> movederror_mass;
    movederror_objptslist_list.resize(objPts_origin_record.size());
    movederror_median_indexinnextleft.resize(objPts_origin_record.size());
    movederror_median.resize(objPts_origin_record.size());
    movederror_mass.resize(objPts_origin_record.size());
    for(int i=0;i<objPts_origin_record.size();i++){
        
        for(int j=0;j<objPts_origin_record[i].size();j++){
            cv::Point3f everypoint = objPts_origin_record[i][j];
            float movederror_objpts = std::sqrt(std::pow((everypoint.x-objPts_next_record[i][j].x),2)+std::pow((everypoint.y-objPts_next_record[i][j].y),2)+std::pow((everypoint.z-objPts_next_record[i][j].z),2));
            //runqiu: 误差超过10m的都是outlier, 因为车子的速度不可能这么快
            if(movederror_objpts<=10)
                movederror_objptslist_list[i].push_back(movederror_objpts);
        }
        if(movederror_objptslist_list[i].empty()){
            movederror_median_indexinnextleft[i] = -1;
            movederror_median[i] = -1;
            movederror_mass[i]=-1;
            continue;
        }
        //runqiu:求中位数，并返回它在当前帧左图中的索引，偶数个元素时取小端（激进选择）
        vector<float> copy;//runqiu:做一个movederror_objptslist_list的拷贝用于排序
        std::copy(movederror_objptslist_list[i].begin(), movederror_objptslist_list[i].end(), std::back_inserter(copy));
        std::sort(copy.begin(), copy.end());
        float themedian;
        if(copy.size()%2==0){
            themedian = copy[copy.size()/2-1];
            std::vector<float>::iterator it = std::find(movederror_objptslist_list[i].begin(), movederror_objptslist_list[i].end(), themedian);
            int median_indexinlist = std::distance(movederror_objptslist_list[i].begin(), it);
            movederror_median_indexinnextleft[i] = objPts_next_record_indexinleft[i][median_indexinlist];
            movederror_median[i] = movederror_objptslist_list[i][median_indexinlist];
            if(i<(objPts_origin_record.size()-1)){
            //     cout<<ref_frame.masksInLeft.labels[i]<<" in curr: x="<<to_string(objPts_next_record[i][median_indexinlist].x)<<", y="<<to_string(objPts_next_record[i][median_indexinlist].y)<<", z="<<to_string(objPts_next_record[i][median_indexinlist].z)<<endl;
            //     cout<<curr_frame.masksInLeft.labels[i]<<" in ref: x="<<to_string(objPts_origin_record[i][median_indexinlist].x)<<", y="<<to_string(objPts_origin_record[i][median_indexinlist].y)<<", z="<<to_string(objPts_origin_record[i][median_indexinlist].z)<<endl;
            }
            
        }
        else{
            themedian = copy[(copy.size()+1)/2-1];
            std::vector<float>::iterator it = std::find(movederror_objptslist_list[i].begin(), movederror_objptslist_list[i].end(), themedian);
            int median_indexinlist = std::distance(movederror_objptslist_list[i].begin(), it);
            movederror_median_indexinnextleft[i] = objPts_next_record_indexinleft[i][median_indexinlist];
            movederror_median[i] = movederror_objptslist_list[i][median_indexinlist];
            if(i<(objPts_origin_record.size()-1)){
            //     cout<<ref_frame.masksInLeft.labels[i]<<": x="<<to_string(objPts_next_record[i][median_indexinlist].x)<<", y="<<to_string(objPts_next_record[i][median_indexinlist].y)<<", z="<<to_string(objPts_next_record[i][median_indexinlist].z)<<endl;
            //     cout<<curr_frame.masksInLeft.labels[i]<<" in ref: x="<<to_string(objPts_origin_record[i][median_indexinlist].x)<<", y="<<to_string(objPts_origin_record[i][median_indexinlist].y)<<", z="<<to_string(objPts_origin_record[i][median_indexinlist].z)<<endl;
            }
            
        }
    }

    //runqiu:使用高斯分布判断各个物体是否为dynamic，并写入结构体中，注意背景不必写入mask结构体中
    masksSDForThisFrame.bboxes = curr_frame.masksInLeft.bboxes;
    masksSDForThisFrame.labels = curr_frame.masksInLeft.labels;

    //runqiu: delete outliers until median for every objects
    for(int i=0; i<movederror_objptslist_list.size(); i++){
        if(movederror_median[i]==-1)
            continue;//no points in this objects
        // cout<<"object "<<to_string(i)<<" points number-before deleting: "<<to_string(movederror_objptslist_list[i].size())<<endl;
        for(int j=0; j<movederror_objptslist_list[i].size(); j++){
            if(movederror_objptslist_list[i][j]>movederror_median[i]){
                movederror_objptslist_list[i].erase(movederror_objptslist_list[i].begin() + j);
            }

        }
        // cout<<"object "<<to_string(i)<<" points number: "<<to_string(movederror_objptslist_list[i].size())<<endl;//movederror_objptslist_list中误差大于10m的都已被删除，所以中位数不一定位于其中点
    }

    // cout<<"std of bkg: 0.12"<<endl;
    // float stdev=0.12;
    std::vector<float> inlierratio;
    inlierratio.resize(movederror_objptslist_list.size()-1); 
    for(int i=0;i<(movederror_objptslist_list.size()-1);i++){
        if(movederror_objptslist_list[i].empty()){
            inlierratio[i]=-1;
            continue;
        }
        int count=0;
        for(int j=0; j<movederror_objptslist_list[i].size(); j++){
            // cout<<"    object "<<to_string(i)<<" point "<<to_string(j)<<" error: "<<to_string(movederror_objptslist_list[i][j])<<endl;
            if(movederror_objptslist_list[i][j]<=(stdev*3))
                count++;
        }
        inlierratio[i]=count*1.0/movederror_objptslist_list[i].size();
    }
    for(int i=0;i<masksSDForThisFrame.labels.size();i++){
        masksSDForThisFrame.isdynamic.push_back(2);//runqiu:默认所有检出的物体都是not in ref_frame或者lost tracking of the object
    }
    // float dynaThreshold=0.7;
    for(int i=0;i<(inlierratio.size());i++){
        // cout<<"object "<<to_string(i)<<" inlierratio: "<<to_string(inlierratio[i])<<endl;
        if(inlierratio[i]==-1){
            continue;//lost tracking of the object
        }
        if(inlierratio[i]>=dynaThreshold){//runqiu:如果该物体的所有点误差小于三倍std，则认为是static
            
            cv::KeyPoint medianPoint = kpleft_next[movederror_median_indexinnextleft[i]];
            for(int j=0;j<masksSDForThisFrame.bboxes.size();j++){
                if(medianPoint.pt.x>masksSDForThisFrame.bboxes[j].lux && medianPoint.pt.x<masksSDForThisFrame.bboxes[j].rlx && medianPoint.pt.y>masksSDForThisFrame.bboxes[j].luy && medianPoint.pt.y<masksSDForThisFrame.bboxes[j].rly){
                    masksSDForThisFrame.isdynamic[j] = 0;
                }
            }
            continue;
        }
        if(inlierratio[i]<dynaThreshold){//runqiu:则认为是dynamic
            cv::KeyPoint medianPoint = kpleft_next[movederror_median_indexinnextleft[i]];
            for(int j=0;j<masksSDForThisFrame.bboxes.size();j++){
                if(medianPoint.pt.x>masksSDForThisFrame.bboxes[j].lux && medianPoint.pt.x<masksSDForThisFrame.bboxes[j].rlx && medianPoint.pt.y>masksSDForThisFrame.bboxes[j].luy && medianPoint.pt.y<masksSDForThisFrame.bboxes[j].rly){
                    masksSDForThisFrame.isdynamic[j] = 1;
                }
            }
        }

    }
    
    for(int i=0; i<masksSDForThisFrame.isdynamic.size(); i++){//runqiu:0 is static, 1 is dynamic, 2 is lost tracking or not in ref_frame, 3 is lost tracking background
        if(masksSDForThisFrame.isdynamic[i]==0){
            cv::rectangle(imMask, cv::Point(masksSDForThisFrame.bboxes[i].lux,masksSDForThisFrame.bboxes[i].luy), cv::Point(masksSDForThisFrame.bboxes[i].rlx,masksSDForThisFrame.bboxes[i].rly), cv::Scalar(0,0,0), -1);//runqiu: 最后的-1表示填满
            // cout<<"object "<<to_string(i)<<" is: static"<<endl;
        }
        else if(masksSDForThisFrame.isdynamic[i]==1){
            // cout<<"object "<<to_string(i)<<" is: dynamic"<<endl;
        }
        else if(masksSDForThisFrame.isdynamic[i]==2){
            // cout<<"object "<<to_string(i)<<" is: lost tracking or not in ref_frame"<<endl;
        }
        else{
            // cout<<"object "<<to_string(i)<<" is: lost tracking background"<<endl;
        }
    }

    return;
}

} //namespace ORB_SLAM
