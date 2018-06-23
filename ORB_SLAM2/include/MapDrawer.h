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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace ORB_SLAM2
{

class MapDrawer
{
public:
    MapDrawer(Map* pMap, const string &strSettingPath);

    Map* mpMap;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void DrawCurrentDsoCamera(pangolin::OpenGlMatrix &Twc);

    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetCurrentDsoCameraPose(const cv::Mat &Twc, const cv::Mat &Tcw);

    vector<cv::Mat> GetCurrentDsoCameraPose();


    void SetDsoKFsPoses(const std::vector<cv::Mat> &Tcw_vec);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void GetCurrentDsoOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;
    cv::Mat mDsoCameraTwc, mDsoCameraTcw;

    cv::Mat sR_w_dso_from_w_orb;
    cv::Mat t_w_dso_from_w_orb;


    std::mutex mMutexCamera;
    std::mutex mMutexDsoCamera;
    std::mutex mMutexDsoKFs;
    std::mutex mMutexSim3;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
