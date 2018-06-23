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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>


namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void AddDsoMapPointBeforeSim3Alignment(MapPoint* pMP);
    void AddDsoMapPointAfterSim3Alignment(MapPoint* pMP);
    void ClearDsoMapPointsBeforeSim3Alignment();
    void ClearDsoMapPointsAfterSim3Alignment();
    void SetDsoKFsPoses(const std::vector<cv::Mat> &Twc_dso_kf_vec);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    KeyFrame* GetLastGoodKeyFrame();
    std::vector<std::vector<cv::Mat> > GetLastGoodKeyframePositions_orb_dso(const unsigned &nReturn, const unsigned &nSkip);
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetAllDsoMapPointsBeforeSim3Alignment();
    std::vector<MapPoint*> GetAllDsoMapPointsAfterSim3Alignment();
    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<cv::Mat> GetDsoKFsPoses();

    void SetDsoPoseAsFinal();
    void SetOrbPoseAsFinal();
    bool IsDsoPoseFinal();
    bool IsOrbPoseFinal();
    void SetPercentageKFsWithCollinearCovlinks(const float percentage_kfs_with_collinear_covlinks_);
    float GetPercentageKFsWithCollinearCovlinks();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<MapPoint*> mspDsoMapPointsBeforeSim3Alignment;
    std::set<MapPoint*> mspDsoMapPointsAfterSim3Alignment;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;
    std::vector<cv::Mat> mvpDsoKFsPoses;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    bool use_dso_pose_as_final;
    bool use_orb_pose_as_final;
    float percentage_kfs_with_collinear_covlinks;

    std::mutex mMutexMap;
    std::mutex mMutexDsoMap;
    std::mutex mMutexDsoKFs;
    std::mutex mMutexFinalPose;
    std::mutex mMutexPercentageKFsWithCollinearCovlinks;
};

} //namespace ORB_SLAM

#endif // MAP_H
