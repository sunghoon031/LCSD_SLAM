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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():use_dso_pose_as_final(false), use_orb_pose_as_final(false),percentage_kfs_with_collinear_covlinks(0.0f), mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::AddDsoMapPointBeforeSim3Alignment(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexDsoMap);
    mspDsoMapPointsBeforeSim3Alignment.insert(pMP);
}

void Map::AddDsoMapPointAfterSim3Alignment(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexDsoMap);
    mspDsoMapPointsAfterSim3Alignment.insert(pMP);
}


void Map::ClearDsoMapPointsBeforeSim3Alignment()
{
    unique_lock<mutex> lock(mMutexDsoMap);

    mspDsoMapPointsBeforeSim3Alignment.clear();

    // TODO: This only erase the pointer.
    // Delete the MapPoint
    // Problem is in MapDrawer::DrawMapPoints()
}

void Map::ClearDsoMapPointsAfterSim3Alignment()
{
    unique_lock<mutex> lock(mMutexDsoMap);

    mspDsoMapPointsAfterSim3Alignment.clear();

    // TODO: This only erase the pointer.
    // Delete the MapPoint
    // Problem is in MapDrawer::DrawMapPoints()
}

void Map::SetDsoKFsPoses(const std::vector<cv::Mat> &Twc_dso_kf_vec)
{
    unique_lock<mutex> lock(mMutexDsoKFs);
    mvpDsoKFsPoses = Twc_dso_kf_vec;

}

void Map::SetDsoPoseAsFinal()
{
    unique_lock<mutex> lock(mMutexFinalPose);
    use_dso_pose_as_final = true;
    use_orb_pose_as_final = false;
}

void Map::SetOrbPoseAsFinal()
{
    unique_lock<mutex> lock(mMutexFinalPose);
    use_dso_pose_as_final = false;
    use_orb_pose_as_final = true;
}

bool Map::IsDsoPoseFinal()
{
    unique_lock<mutex> lock(mMutexFinalPose);
    return use_dso_pose_as_final;
}

bool Map::IsOrbPoseFinal()
{
    unique_lock<mutex> lock(mMutexFinalPose);
    return use_orb_pose_as_final;
}

void Map::SetPercentageKFsWithCollinearCovlinks(const float percentage_kfs_with_collinear_covlinks_)
{
    unique_lock<mutex> lock(mMutexPercentageKFsWithCollinearCovlinks);
    percentage_kfs_with_collinear_covlinks = percentage_kfs_with_collinear_covlinks_;
}

float Map::GetPercentageKFsWithCollinearCovlinks()
{
    unique_lock<mutex> lock(mMutexPercentageKFsWithCollinearCovlinks);
    return percentage_kfs_with_collinear_covlinks;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

KeyFrame* Map::GetLastGoodKeyFrame()
{
    unique_lock<mutex> lock(mMutexMap);
    KeyFrame* LastGoodKeyFrame = nullptr;

    for (set<KeyFrame*>::reverse_iterator it = mspKeyFrames.rbegin(); it != mspKeyFrames.rend(); ++it )
    {
        if (!(*it)->isBad())
        {
            LastGoodKeyFrame = *it;
            break;
        }
    }

    return LastGoodKeyFrame;
}

vector<vector<cv::Mat> > Map::GetLastGoodKeyframePositions_orb_dso(const unsigned &nReturn, const unsigned &nSkip)
{


    vector<vector<cv::Mat> > output;
    vector<cv::Mat> good_orb_poses, good_dso_poses;
    vector<cv::Mat> output_orb_poses, output_dso_poses;


    unsigned nGoodKFs = 0;

    {
        unique_lock<mutex> lock(mMutexMap);
        for (set<KeyFrame*>::const_iterator it = mspKeyFrames.begin(); it != mspKeyFrames.end(); ++it )
        {
            if (!(*it)->isBad())
            {
                nGoodKFs++;
                good_orb_poses.push_back((*it)->GetTranslation());
                good_dso_poses.push_back((*it)->GetDsoTranslation());
            }
        }
    }

    if(nGoodKFs < nReturn+nSkip)
        return output;


    vector<cv::Mat>::reverse_iterator orb_rbegin = good_orb_poses.rbegin();
    vector<cv::Mat>::reverse_iterator dso_rbegin = good_dso_poses.rbegin();
    advance(orb_rbegin, nSkip);
    advance(dso_rbegin, nSkip);

    for (vector<cv::Mat>::reverse_iterator it = orb_rbegin; it !=good_orb_poses.rend(); ++it)
    {
        if (output_orb_poses.size() == nReturn) break;

        output_orb_poses.push_back(*it);
    }


    for (vector<cv::Mat>::reverse_iterator it = dso_rbegin; it !=good_dso_poses.rend(); ++it)
    {
        if (output_dso_poses.size() == nReturn) break;

        output_dso_poses.push_back(*it);
    }

    output.push_back(output_orb_poses);
    output.push_back(output_dso_poses);


    return output;

}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

vector<MapPoint*> Map::GetAllDsoMapPointsBeforeSim3Alignment()
{
    unique_lock<mutex> lock(mMutexDsoMap);
    vector<MapPoint*> empty_vec;

    if (mspDsoMapPointsBeforeSim3Alignment.empty())
        return empty_vec;
    else
        return vector<MapPoint*>(mspDsoMapPointsBeforeSim3Alignment.begin(),mspDsoMapPointsBeforeSim3Alignment.end());
}

vector<MapPoint*> Map::GetAllDsoMapPointsAfterSim3Alignment()
{
    unique_lock<mutex> lock(mMutexDsoMap);
    vector<MapPoint*> empty_vec;

    if (mspDsoMapPointsAfterSim3Alignment.empty())
        return empty_vec;
    else
        return vector<MapPoint*>(mspDsoMapPointsAfterSim3Alignment.begin(),mspDsoMapPointsAfterSim3Alignment.end());
}

vector<cv::Mat> Map::GetDsoKFsPoses()
{
    unique_lock<mutex> lock(mMutexDsoKFs);
    return mvpDsoKFsPoses;
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    for(set<MapPoint*>::iterator sit=mspDsoMapPointsBeforeSim3Alignment.begin(), send=mspDsoMapPointsBeforeSim3Alignment.end(); sit!=send; sit++)
            delete *sit;

    for(set<MapPoint*>::iterator sit=mspDsoMapPointsAfterSim3Alignment.begin(), send=mspDsoMapPointsAfterSim3Alignment.end(); sit!=send; sit++)
            delete *sit;

    mspDsoMapPointsBeforeSim3Alignment.clear();
    mspDsoMapPointsAfterSim3Alignment.clear();
    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
