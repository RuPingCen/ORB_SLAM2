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

#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM2
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}

int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)
{
    int nmatches=0;

    const bool bFactor = th!=1.0;

    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];
        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;

        const int &nPredictedLevel = pMP->mnTrackScaleLevel;

        // The size of the window will depend on the viewing direction
        float r = RadiusByViewingCos(pMP->mTrackViewCos);

        if(bFactor)
            r*=th;

        const vector<size_t> vIndices =
                F.GetFeaturesInArea(pMP->mTrackProjX,pMP->mTrackProjY,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP->GetDescriptor();

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            if(F.mvpMapPoints[idx])
                if(F.mvpMapPoints[idx]->Observations()>0)
                    continue;

            if(F.mvuRight[idx]>0)
            {
                const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                if(er>r*F.mvScaleFactors[nPredictedLevel])
                    continue;
            }

            const cv::Mat &d = F.mDescriptors.row(idx);

            const int dist = DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = F.mvKeysUn[idx].octave;
                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = F.mvKeysUn[idx].octave;
                bestDist2=dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)
        {
            if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                continue;

            F.mvpMapPoints[bestIdx]=pMP;
            nmatches++;
        }
    }

    return nmatches;
}

float ORBmatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}


bool ORBmatcher::CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12,const KeyFrame* pKF2)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;

    const float dsqr = num*num/den;

    return dsqr<3.84*pKF2->mvLevelSigma2[kp2.octave];
}

int ORBmatcher::SearchByBoW(KeyFrame* pKF,Frame &F, vector<MapPoint*> &vpMapPointMatches)
{
    const vector<MapPoint*> vpMapPointsKF = pKF->GetMapPointMatches();

    vpMapPointMatches = vector<MapPoint*>(F.N,static_cast<MapPoint*>(NULL));

    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;

    int nmatches=0;

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

    while(KFit != KFend && Fit != Fend)
    {
        if(KFit->first == Fit->first)
        {
            const vector<unsigned int> vIndicesKF = KFit->second;
            const vector<unsigned int> vIndicesF = Fit->second;

            for(size_t iKF=0; iKF<vIndicesKF.size(); iKF++)
            {
                const unsigned int realIdxKF = vIndicesKF[iKF];

                MapPoint* pMP = vpMapPointsKF[realIdxKF];

                if(!pMP)
                    continue;

                if(pMP->isBad())
                    continue;                

                const cv::Mat &dKF= pKF->mDescriptors.row(realIdxKF);

                int bestDist1=256;
                int bestIdxF =-1 ;
                int bestDist2=256;

                for(size_t iF=0; iF<vIndicesF.size(); iF++)
                {
                    const unsigned int realIdxF = vIndicesF[iF];

                    if(vpMapPointMatches[realIdxF])
                        continue;

                    const cv::Mat &dF = F.mDescriptors.row(realIdxF);

                    const int dist =  DescriptorDistance(dKF,dF);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdxF=realIdxF;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1<=TH_LOW)
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMapPointMatches[bestIdxF]=pMP;

                        const cv::KeyPoint &kp = pKF->mvKeysUn[realIdxKF];

                        if(mbCheckOrientation)
                        {
                            float rot = kp.angle-F.mvKeys[bestIdxF].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(bestIdxF);
                        }
                        nmatches++;
                    }
                }

            }

            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }


    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMapPointMatches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}

int ORBmatcher::SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // Decompose Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

    int nmatches=0;

    // For each Candidate MapPoint Project and Match
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0)
            continue;

        // Project into Image
        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale invariance region of the point
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist,pKF);

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            if(vpMatched[idx])
                continue;

            const int &kpLevel= pKF->mvKeysUn[idx].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_LOW)
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }

    }

    return nmatches;
}

int ORBmatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        if(level1>0)
            continue;

        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;

        cv::Mat d1 = F1.mDescriptors.row(i1);

        int bestDist = INT_MAX;
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            cv::Mat d2 = F2.mDescriptors.row(i2);

            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        if(bestDist<=TH_LOW)
        {
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;

                if(mbCheckOrientation)
                {
                    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }

    }

    //Update prev matched
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}

int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
{
    const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const cv::Mat &Descriptors1 = pKF1->mDescriptors;

    const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKeysUn;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const cv::Mat &Descriptors2 = pKF2->mDescriptors;

    vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(),false);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while(f1it != f1end && f2it != f2end)
    {
        if(f1it->first == f2it->first)
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];

                MapPoint* pMP1 = vpMapPoints1[idx1];
                if(!pMP1)
                    continue;
                if(pMP1->isBad())
                    continue;

                const cv::Mat &d1 = Descriptors1.row(idx1);

                int bestDist1=256;
                int bestIdx2 =-1 ;
                int bestDist2=256;

                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    if(vbMatched2[idx2] || !pMP2)
                        continue;

                    if(pMP2->isBad())
                        continue;

                    const cv::Mat &d2 = Descriptors2.row(idx2);

                    int dist = DescriptorDistance(d1,d2);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdx2=idx2;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1<TH_LOW)
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMatches12[idx1]=vpMapPoints2[bestIdx2];
                        vbMatched2[bestIdx2]=true;

                        if(mbCheckOrientation)
                        {
                            float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMatches12[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}

int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                                       vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo)
{    
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

    //Compute epipole in second image
    cv::Mat Cw = pKF1->GetCameraCenter();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();
    cv::Mat C2 = R2w*Cw+t2w;
    const float invz = 1.0f/C2.at<float>(2);
    const float ex =pKF2->fx*C2.at<float>(0)*invz+pKF2->cx;
    const float ey =pKF2->fy*C2.at<float>(1)*invz+pKF2->cy;

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node

    int nmatches=0;
    vector<bool> vbMatched2(pKF2->N,false);
    vector<int> vMatches12(pKF1->N,-1);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while(f1it!=f1end && f2it!=f2end)
    {
        if(f1it->first == f2it->first)
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];
                
                MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
                
                // If there is already a MapPoint skip
                if(pMP1)
                    continue;

                const bool bStereo1 = pKF1->mvuRight[idx1]>=0;

                if(bOnlyStereo)
                    if(!bStereo1)
                        continue;
                
                const cv::KeyPoint &kp1 = pKF1->mvKeysUn[idx1];
                
                const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);
                
                int bestDist = TH_LOW;
                int bestIdx2 = -1;
                
                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    size_t idx2 = f2it->second[i2];
                    
                    MapPoint* pMP2 = pKF2->GetMapPoint(idx2);
                    
                    // If we have already matched or there is a MapPoint skip
                    if(vbMatched2[idx2] || pMP2)
                        continue;

                    const bool bStereo2 = pKF2->mvuRight[idx2]>=0;

                    if(bOnlyStereo)
                        if(!bStereo2)
                            continue;
                    
                    const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);
                    
                    const int dist = DescriptorDistance(d1,d2);
                    
                    if(dist>TH_LOW || dist>bestDist)
                        continue;

                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

                    if(!bStereo1 && !bStereo2)
                    {
                        const float distex = ex-kp2.pt.x;
                        const float distey = ey-kp2.pt.y;
                        if(distex*distex+distey*distey<100*pKF2->mvScaleFactors[kp2.octave])
                            continue;
                    }

                    if(CheckDistEpipolarLine(kp1,kp2,F12,pKF2))
                    {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }
                
                if(bestIdx2>=0)
                {
                    const cv::KeyPoint &kp2 = pKF2->mvKeysUn[bestIdx2];
                    vMatches12[idx1]=bestIdx2;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = kp1.angle-kp2.angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(idx1);
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vMatches12[rotHist[i][j]]=-1;
                nmatches--;
            }
        }

    }

    vMatchedPairs.clear();
    vMatchedPairs.reserve(nmatches);

    for(size_t i=0, iend=vMatches12.size(); i<iend; i++)
    {
        if(vMatches12[i]<0)
            continue;
        vMatchedPairs.push_back(make_pair(i,vMatches12[i]));
    }

    return nmatches;
}

int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th)
{
    cv::Mat Rcw = pKF->GetRotation();
    cv::Mat tcw = pKF->GetTranslation();

    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;
    const float &bf = pKF->mbf;

    cv::Mat Ow = pKF->GetCameraCenter();

    int nFused=0;

    const int nMPs = vpMapPoints.size();

    for(int i=0; i<nMPs; i++)
    {
        MapPoint* pMP = vpMapPoints[i];

        if(!pMP)
            continue;

        if(pMP->isBad() || pMP->IsInKeyFrame(pKF))
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc = Rcw*p3Dw + tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0f)
            continue;

        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        const float ur = u-bf*invz;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist3D = cv::norm(PO);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = 256;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF->mvKeysUn[idx];

            const int &kpLevel= kp.octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            if(pKF->mvuRight[idx]>=0)
            {
                // Check reprojection error in stereo
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float &kpr = pKF->mvuRight[idx];
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float er = ur-kpr;
                const float e2 = ex*ex+ey*ey+er*er;

                if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8)
                    continue;
            }
            else
            {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float ex = u-kpx;
                const float ey = v-kpy;
                const float e2 = ex*ex+ey*ey;

                if(e2*pKF->mvInvLevelSigma2[kpLevel]>5.99)
                    continue;
            }

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                {
                    if(pMPinKF->Observations()>pMP->Observations())
                        pMP->Replace(pMPinKF);
                    else
                        pMPinKF->Replace(pMP);
                }
            }
            else
            {
                pMP->AddObservation(pKF,bestIdx);
                pKF->AddMapPoint(pMP,bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}

int ORBmatcher::Fuse(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)
{
    // Get Calibration Parameters for later projection
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;

    // Decompose Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

    int nFused=0;

    const int nPoints = vpPoints.size();

    // For each candidate MapPoint project and match
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0f)
            continue;

        // Project into Image
        const float invz = 1.0/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale pyramid of the image
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist3D = cv::norm(PO);

        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        // Compute predicted scale level
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF);

        // Search in a radius
        const float radius = th*pKF->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t idx = *vit;
            const int &kpLevel = pKF->mvKeysUn[idx].octave;

            if(kpLevel<nPredictedLevel-1 || kpLevel>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF->mDescriptors.row(idx);

            int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                    vpReplacePoint[iMP] = pMPinKF;
            }
            else
            {
                pMP->AddObservation(pKF,bestIdx);
                pKF->AddMapPoint(pMP,bestIdx);
            }
            nFused++;
        }
    }

    return nFused;
}

int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint*> &vpMatches12,
                             const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th)
{
    const float &fx = pKF1->fx;
    const float &fy = pKF1->fy;
    const float &cx = pKF1->cx;
    const float &cy = pKF1->cy;

    // Camera 1 from world
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();

    //Camera 2 from world
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    //Transformation between cameras
    cv::Mat sR12 = s12*R12;
    cv::Mat sR21 = (1.0/s12)*R12.t();
    cv::Mat t21 = -sR21*t12;

    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();

    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const int N2 = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

    for(int i=0; i<N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;
            int idx2 = pMP->GetIndexInKeyFrame(pKF2);
            if(idx2>=0 && idx2<N2)
                vbAlreadyMatched2[idx2]=true;
        }
    }

    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);

    // Transform from KF1 to KF2 and search
    for(int i1=0; i1<N1; i1++)
    {
        MapPoint* pMP = vpMapPoints1[i1];

        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc1 = R1w*p3Dw + t1w;
        cv::Mat p3Dc2 = sR21*p3Dc1 + t21;

        // Depth must be positive
        if(p3Dc2.at<float>(2)<0.0)
            continue;

        const float invz = 1.0/p3Dc2.at<float>(2);
        const float x = p3Dc2.at<float>(0)*invz;
        const float y = p3Dc2.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF2->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc2);

        // Depth must be inside the scale invariance region
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF2);

        // Search in a radius
        const float radius = th*pKF2->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF2->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF2->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch1[i1]=bestIdx;
        }
    }

    // Transform from KF2 to KF2 and search
    for(int i2=0; i2<N2; i2++)
    {
        MapPoint* pMP = vpMapPoints2[i2];

        if(!pMP || vbAlreadyMatched2[i2])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc2 = R2w*p3Dw + t2w;
        cv::Mat p3Dc1 = sR12*p3Dc2 + t12;

        // Depth must be positive
        if(p3Dc1.at<float>(2)<0.0)
            continue;

        const float invz = 1.0/p3Dc1.at<float>(2);
        const float x = p3Dc1.at<float>(0)*invz;
        const float y = p3Dc1.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF1->IsInImage(u,v))
            continue;

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc1);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Compute predicted octave
        const int nPredictedLevel = pMP->PredictScale(dist3D,pKF1);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        const float radius = th*pKF1->mvScaleFactors[nPredictedLevel];

        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const cv::Mat dMP = pMP->GetDescriptor();

        int bestDist = INT_MAX;
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            const cv::KeyPoint &kp = pKF1->mvKeysUn[idx];

            if(kp.octave<nPredictedLevel-1 || kp.octave>nPredictedLevel)
                continue;

            const cv::Mat &dKF = pKF1->mDescriptors.row(idx);

            const int dist = DescriptorDistance(dMP,dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch2[i2]=bestIdx;
        }
    }

    // Check agreement
    int nFound = 0;

    for(int i1=0; i1<N1; i1++)
    {
        int idx2 = vnMatch1[i1];

        if(idx2>=0)
        {
            int idx1 = vnMatch2[idx2];
            if(idx1==i1)
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}

int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);

    const cv::Mat twc = -Rcw.t()*tcw;

    const cv::Mat Rlw = LastFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tlw = LastFrame.mTcw.rowRange(0,3).col(3);

    const cv::Mat tlc = Rlw*twc+tlw;

    const bool bForward = tlc.at<float>(2)>CurrentFrame.mb && !bMono;
    const bool bBackward = -tlc.at<float>(2)>CurrentFrame.mb && !bMono;

    for(int i=0; i<LastFrame.N; i++)
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];

        if(pMP)
        {
            if(!LastFrame.mvbOutlier[i])
            {
                // Project
                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw*x3Dw+tcw;

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);

                if(invzc<0)
                    continue;

                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

                int nLastOctave = LastFrame.mvKeys[i].octave;

                // Search in a window. Size depends on scale
                float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];

                vector<size_t> vIndices2;

                if(bForward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave);
                else if(bBackward)
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0, nLastOctave);
                else
                    vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, nLastOctave-1, nLastOctave+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        if(CurrentFrame.mvpMapPoints[i2]->Observations()>0)
                            continue;

                    if(CurrentFrame.mvuRight[i2]>0)
                    {
                        const float ur = u - CurrentFrame.mbf*invzc;
                        const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                        if(er>radius)
                            continue;
                    }

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=TH_HIGH)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }
            }
        }
    }

    //Apply rotation consistency
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

/**
 * @brief 对上一帧每个3D点通过投影作为光流跟踪的初始值。从而实现当前帧CurrentFrame对上一帧LastFrame 3D点的匹配跟踪。用于tracking中前后帧跟踪
 *
 * 上一帧中包含了MapPoints，对这些MapPoints进行tracking，由此增加当前帧的MapPoints \n
 * 1. 将上一帧的MapPoints投影到当前帧(根据速度模型可以估计当前帧的Tcw)
 * 2. 根据光流进行匹配，以及最终的方向投票机制进行剔除
 * @param  CurrentFrame 当前帧
 * @param  LastFrame    上一帧
 * @return              成功匹配的数量
 * @see SearchByProjection()
 */
int ORBmatcher::SearchByOpticalFlow(Frame &CurrentFrame, const Frame &LastFrame)
{
    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    const cv::Mat twc = -Rcw.t()*tcw;

    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_last, kps_current;
    vector<int> vMapPointIdx;//第几个有效点对应第几个特征点
    vMapPointIdx.reserve(LastFrame.N);
    
    for(int i=0; i<LastFrame.N; i++)//对上以帧的每个特征点
    {
        MapPoint* pMP = LastFrame.mvpMapPoints[i];//获取地标

        if(pMP)//如果特征点对应的地标不为空
        {
            if(!LastFrame.mvbOutlier[i])//并且不是外点
            {
                // Project
	        // 对上一帧有效的MapPoints进行跟踪
                cv::Mat x3Dw = pMP->GetWorldPos();//获取世界坐标
                cv::Mat x3Dc = Rcw*x3Dw+tcw;//得到当前相机下的相机坐标

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);//逆深度

                if(invzc<0)
                    continue;

                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;//地标点在当前相机的像素坐标系
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)//检查边界
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

	        vMapPointIdx.push_back(i);//为特征点添加索引
			kps_last.push_back(LastFrame.mvKeys[i].pt);
			kps_current.push_back(cv::Point2f(u, v));
            }
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK( LastFrame.mLeftImage,CurrentFrame.mLeftImage, kps_last,
			      kps_current, status, error, cv::Size(11, 11), 3,
			      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01),
			      cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    CurrentFrame.mvKeys.clear();
    CurrentFrame.mvpMapPoints.clear();
    
    vector<cv::KeyPoint> prev_keypoints;//用于显示//上一帧对应的关键点
    
    for (size_t i = 0; i < status.size(); ++i) 
    {
        if (status[i]) 
	{
			cv::KeyPoint  kp(kps_current[i], LastFrame.mvKeys[vMapPointIdx[i]].size, LastFrame.mvKeys[vMapPointIdx[i]].angle, 
					LastFrame.mvKeys[vMapPointIdx[i]].response,LastFrame.mvKeys[vMapPointIdx[i]].octave,
					LastFrame.mvKeys[vMapPointIdx[i]].class_id);

			CurrentFrame.mvKeys.push_back(kp);
			CurrentFrame.mvpMapPoints.push_back( LastFrame.mvpMapPoints[vMapPointIdx[i]]);//为当前帧添加MapPoint

			prev_keypoints.push_back(LastFrame.mvKeys[vMapPointIdx[i]]);//用于显示

			num_good_pts++;
        }
    }

    ////////////////////////////////////////////显示匹配/////////////////////////////////////////////
//     cv::Mat outimg1,outimg2;
//     cv::cvtColor ( CurrentFrame.left_img_, outimg1, CV_GRAY2BGR );
//     cv::cvtColor ( LastFrame.left_img_, outimg2, CV_GRAY2BGR );
// 
//     vector<cv::DMatch> matches;
//     matches.resize(prev_keypoints.size());
//     for(unsigned int n=0;n< prev_keypoints.size();n++)
//     {
// 	  matches[n].queryIdx=n;
// 	  matches[n].trainIdx=n;
//     }
//     cv::Mat img_match;//光流匹配点图
//     cv::drawMatches( outimg2, prev_keypoints, outimg1, CurrentFrame.mvKeys, matches, img_match);
//     cv::namedWindow("光流匹配点对",0);
//     cv::imshow("光流匹配点对", img_match);
//     cv::waitKey(10);
    //////////////////////////////////////////////////////////////////

    //////////////制作Frame中的数据////////////////////////////////////////////////
     CurrentFrame.N = CurrentFrame.mvKeys.size();
     CurrentFrame.mvbOutlier = vector<bool>(CurrentFrame.N,false);

    // Undistort特征点，只对左目进行了矫正，（因为要求输入的图像已经进行极线校正？）
    CurrentFrame.UndistortKeyPoints();
    //设置非双目信息，我们在光流跟踪的时候仅仅利用左目去跟踪，并没有使用到右目的信息
    CurrentFrame.mvuRight = vector<float>(CurrentFrame.N,-1.0f);
    CurrentFrame.mvDepth = vector<float>(CurrentFrame.N,-1.0f);
    // 计算双目间的匹配, 匹配成功的特征点会计算其深度

    return num_good_pts;
}
 int ORBmatcher::SearchByOpticalFlow(Frame &CurrentFrame, KeyFrame *pKF)
 {
	   const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    const cv::Mat twc = -Rcw.t()*tcw;

    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_last, kps_current;
    vector<int> vMapPointIdx;//第几个有效点对应第几个特征点
    vMapPointIdx.reserve(pKF->N);
 
    for(int i=0; i<pKF->N; i++)//对上以帧的每个特征点
    {
        MapPoint* pMP = pKF->mvpMapPoints[i];//获取地标

        if(pMP)//如果特征点对应的地标不为空
        {
          //  if(!pKF->mvbOutlier[i])//并且不是外点
            {
                // Project
	        // 对上一帧有效的MapPoints进行跟踪
                cv::Mat x3Dw = pMP->GetWorldPos();//获取世界坐标
                cv::Mat x3Dc = Rcw*x3Dw+tcw;//得到当前相机下的相机坐标

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);//逆深度

                if(invzc<0)
                    continue;

                float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;//地标点在当前相机的像素坐标系
                float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)//检查边界
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

	        vMapPointIdx.push_back(i);//为特征点添加索引
		kps_last.push_back(pKF->mvKeys[i].pt);
		kps_current.push_back(cv::Point2f(u, v));
            }
        }
    }
  
    std::vector<uchar> status;
    cv::Mat error;
 
   // cv::calcOpticalFlowPyrLK( LastFrame.mLeftImage,CurrentFrame.mLeftImage, kps_last,
	//		      kps_current, status, error, cv::Size(11, 11), 8,
	//		      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01),
	//		      cv::OPTFLOW_USE_INITIAL_FLOW);
 
    int num_good_pts = 0;
    CurrentFrame.mvKeys.clear();
    CurrentFrame.mvpMapPoints.clear();

    vector<cv::KeyPoint> prev_keypoints;//用于显示//上一帧对应的关键点
    
    for (size_t i = 0; i < status.size(); ++i) 
    {
        if (status[i]) 
		{
			cv::KeyPoint  kp(kps_current[i], pKF->mvKeys[vMapPointIdx[i]].size, pKF->mvKeys[vMapPointIdx[i]].angle, 
					pKF->mvKeys[vMapPointIdx[i]].response,pKF->mvKeys[vMapPointIdx[i]].octave,
					pKF->mvKeys[vMapPointIdx[i]].class_id);
			
			CurrentFrame.mvKeys.push_back(kp);
			CurrentFrame.mvpMapPoints.push_back( pKF->mvpMapPoints[vMapPointIdx[i]]);//为当前帧添加MapPoint
			
			prev_keypoints.push_back(pKF->mvKeys[vMapPointIdx[i]]);//用于显示
			
				num_good_pts++;
        }
    }
 
 
    //////////////制作Frame中的数据////////////////////////////////////////////////
     CurrentFrame.N = CurrentFrame.mvKeys.size();
     CurrentFrame.mvbOutlier = vector<bool>(CurrentFrame.N,false);

    // Undistort特征点，只对左目进行了矫正，（因为要求输入的图像已经进行极线校正？）
    CurrentFrame.UndistortKeyPoints();
    //设置非双目信息，我们在光流跟踪的时候仅仅利用左目去跟踪，并没有使用到右目的信息
    CurrentFrame.mvuRight = vector<float>(CurrentFrame.N,-1.0f);
    CurrentFrame.mvDepth = vector<float>(CurrentFrame.N,-1.0f);
    // 计算双目间的匹配, 匹配成功的特征点会计算其深度

    return num_good_pts;
}

/*******************************MSCKF 相关函数**************************************************************/
/**
 * @brief 输入左右目点集进行匹配　先使用光流匹配，再使用极线匹配
 * @param  cam0_points  左目相机的特征点
 * @param  cam1_points  右目相机的特征点
 * @return              　
 * @see stereoMatch()
 */
// void ORBmatcher::stereoMatch( const vector<cv::Point2f>& cam0_points,
//     vector<cv::Point2f>& cam1_points, vector<unsigned char>& inlier_markers) 
// {
//   if (cam0_points.size() == 0) return;
//   if(cam1_points.size() == 0) return;
// //   if(cam1_points.size() == 0) {
// //     // Initialize cam1_points by projecting cam0_points to cam1 using the
// //     // rotation from stereo extrinsics
// //     const cv::Matx33d R_cam0_cam1 = R_cam1_imu.t() * R_cam0_imu;
// //     vector<cv::Point2f> cam0_points_undistorted;
// //     cv::undistortPoints(cam0_points, cam0_intrinsics, cam0_distortion_model,
// //                     cam0_distortion_coeffs, cam0_points_undistorted,
// //                     R_cam0_cam1);
// //     cam1_points = cv::distortPoints(cam0_points_undistorted, cam1_intrinsics,
// //                                 cam1_distortion_model, cam1_distortion_coeffs);
// //   }
// 
//   // Track features using LK optical flow method.
//   cv::calcOpticalFlowPyrLK(curr_cam0_pyramid_, curr_cam1_pyramid_,
//       cam0_points, cam1_points,
//       inlier_markers, noArray(),
//        cv::Size(processor_config.patch_size, processor_config.patch_size),
//       processor_config.pyramid_levels,
//       TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
//                    processor_config.max_iteration,
//                    processor_config.track_precision),
//       cv::OPTFLOW_USE_INITIAL_FLOW);
// 
//   // Mark those tracked points out of the image region
//   // as untracked.
//   for (int i = 0; i < cam1_points.size(); ++i) {
//     if (inlier_markers[i] == 0) continue;
//     if (cam1_points[i].y < 0 ||
//         cam1_points[i].y > cam1_curr_img_ptr->image.rows-1 ||
//         cam1_points[i].x < 0 ||
//         cam1_points[i].x > cam1_curr_img_ptr->image.cols-1)
//       inlier_markers[i] = 0;
//   }
// 
//   // Compute the relative rotation between the cam0
//   // frame and cam1 frame.
//   const cv::Matx33d R_cam0_cam1 = R_cam1_imu.t() * R_cam0_imu;
//   const cv::Vec3d t_cam0_cam1 = R_cam1_imu.t() * (t_cam0_imu-t_cam1_imu);
//   // Compute the essential matrix.
//   const cv::Matx33d t_cam0_cam1_hat(
//       0.0, -t_cam0_cam1[2], t_cam0_cam1[1],
//       t_cam0_cam1[2], 0.0, -t_cam0_cam1[0],
//       -t_cam0_cam1[1], t_cam0_cam1[0], 0.0);
//   const cv::Matx33d E = t_cam0_cam1_hat * R_cam0_cam1;
// 
//   // Further remove outliers based on the known
//   // essential matrix.
//   vector<cv::Point2f> cam0_points_undistorted(0);
//   vector<cv::Point2f> cam1_points_undistorted(0);
//    cv::undistortPoints( cam0_points, cam0_intrinsics, cam0_distortion_model,
//       cam0_distortion_coeffs, cam0_points_undistorted);
//    cv::undistortPoints( cam1_points, cam1_intrinsics, cam1_distortion_model,
//       cam1_distortion_coeffs, cam1_points_undistorted);
// 
//   double norm_pixel_unit = 4.0 / (
//       cam0_intrinsics[0]+cam0_intrinsics[1]+
//       cam1_intrinsics[0]+cam1_intrinsics[1]);
// 
//   for (int i = 0; i < cam0_points_undistorted.size(); ++i) {
//     if (inlier_markers[i] == 0) continue;
//     cv::Vec3d pt0(cam0_points_undistorted[i].x,
//         cam0_points_undistorted[i].y, 1.0);
//     cv::Vec3d pt1(cam1_points_undistorted[i].x,
//         cam1_points_undistorted[i].y, 1.0);
//     cv::Vec3d epipolar_line = E * pt0;
//     double error = fabs((pt1.t() * epipolar_line)[0]) / sqrt(
//         epipolar_line[0]*epipolar_line[0]+
//         epipolar_line[1]*epipolar_line[1]);
//     if (error > processor_config.stereo_threshold*norm_pixel_unit)
//       inlier_markers[i] = 0;
//   }
// 
//   return;
// }
 /**
 * @brief  使用在ＭＳＣＫＦ里面描述的 SearchByTwoPointRansac　算法
　
 * @param  CurrentFrame 当前帧
 * @param  LastFrame    上一帧
 * @return              成功匹配的数量
 * @see SearchByProjection()
 */
//  int ORBmatcher::SearchByTwoPointRansac(Frame &CurrentFrame, const Frame &LastFrame)
//  {
// 	 // Size of each grid.
//   static int grid_height =LastFrame.mLeftImage.rows;
//     cam0_curr_img_ptr->image.rows / processor_config.grid_row;
//   static int grid_width =
//     cam0_curr_img_ptr->image.cols / processor_config.grid_col;
// 
//   // Compute a rough relative rotation which takes a vector
//   // from the previous frame to the current frame.
//   Matx33f cam0_R_p_c;
//   Matx33f cam1_R_p_c;
//   integrateImuData(cam0_R_p_c, cam1_R_p_c);
// 
//   // Organize the features in the previous image.
//   vector<FeatureIDType> prev_ids(0);
//   vector<int> prev_lifetime(0);
//   vector<Point2f> prev_cam0_points(0);
//   vector<Point2f> prev_cam1_points(0);
// 
//   for (const auto& item : *prev_features_ptr) {
//     for (const auto& prev_feature : item.second) {
//       prev_ids.push_back(prev_feature.id);
//       prev_lifetime.push_back(prev_feature.lifetime);
//       prev_cam0_points.push_back(prev_feature.cam0_point);
//       prev_cam1_points.push_back(prev_feature.cam1_point);
//     }
//   }
// 
//   // Number of the features before tracking.
//  size_t before_tracking = prev_cam0_points.size();
// 
//   // Abort tracking if there is no features in
//   // the previous frame.
//   if (prev_ids.size() == 0) return;
// 
//   // Track features using LK optical flow method.
//   vector<cv::Point2f> curr_cam0_points(0);
//   vector<unsigned char> track_inliers(0);
//       cv::Mat error;
//   //predictFeatureTracking(prev_cam0_points, cam0_R_p_c, cam0_intrinsics, curr_cam0_points);
//   cv::calcOpticalFlowPyrLK( LastFrame.mLeftImage,CurrentFrame.mLeftImage, prev_cam0_points, 
// 				curr_cam0_points, track_inliers, error,cv::Size(11, 11), 3,
// 				cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,0.01),
// 				cv::OPTFLOW_USE_INITIAL_FLOW);
// 
//   // Mark those tracked points out of the image region
//   // as untracked.
//   for (int i = 0; i < curr_cam0_points.size(); ++i) {
//     if (track_inliers[i] == 0) continue;
//     if (curr_cam0_points[i].y < 0 ||
//         curr_cam0_points[i].y >LastFrame.mLeftImage.rows-1 ||
//         curr_cam0_points[i].x < 0 ||
//         curr_cam0_points[i].x >LastFrame.mLeftImage.cols-1)
//       track_inliers[i] = 0;
//   }
// 
//   // Collect the tracked points.
//   vector<FeatureIDType> prev_tracked_ids(0);
//   vector<int> prev_tracked_lifetime(0);
//   vector<Point2f> prev_tracked_cam0_points(0);
//   vector<Point2f> prev_tracked_cam1_points(0);
//   vector<Point2f> curr_tracked_cam0_points(0);
// 
//   removeUnmarkedElements(prev_ids, track_inliers, prev_tracked_ids);
//   removeUnmarkedElements(prev_lifetime, track_inliers, prev_tracked_lifetime);
//   removeUnmarkedElements(prev_cam0_points, track_inliers, prev_tracked_cam0_points);
//   removeUnmarkedElements(prev_cam1_points, track_inliers, prev_tracked_cam1_points);
//   removeUnmarkedElements(curr_cam0_points, track_inliers, curr_tracked_cam0_points);
// 
//   // Number of features left after tracking.
//   size_t after_tracking = curr_tracked_cam0_points.size();
// 
// 
//   // Outlier removal involves three steps, which forms a close
//   // loop between the previous and current frames of cam0 (left)
//   // and cam1 (right). Assuming the stereo matching between the
//   // previous cam0 and cam1 images are correct, the three steps are:
//   //
//   // prev frames cam0 ----------> cam1
//   //              |                |
//   //              |ransac          |ransac
//   //              |   stereo match |
//   // curr frames cam0 ----------> cam1
//   //
//   // 1) Stereo matching between current images of cam0 and cam1.
//   // 2) RANSAC between previous and current images of cam0.
//   // 3) RANSAC between previous and current images of cam1.
//   //
//   // For Step 3, tracking between the images is no longer needed.
//   // The stereo matching results are directly used in the RANSAC.
// 
//   // Step 1: stereo matching.
//   vector<Point2f> curr_cam1_points(0);
//   vector<unsigned char> match_inliers(0);
//   stereoMatch(curr_tracked_cam0_points, curr_cam1_points, match_inliers);
// 
//   vector<FeatureIDType> prev_matched_ids(0);
//   vector<int> prev_matched_lifetime(0);
//   vector<Point2f> prev_matched_cam0_points(0);
//   vector<Point2f> prev_matched_cam1_points(0);
//   vector<Point2f> curr_matched_cam0_points(0);
//   vector<Point2f> curr_matched_cam1_points(0);
// 
//   removeUnmarkedElements(prev_tracked_ids, match_inliers, prev_matched_ids);
//   removeUnmarkedElements(prev_tracked_lifetime, match_inliers, prev_matched_lifetime);
//   removeUnmarkedElements(prev_tracked_cam0_points, match_inliers, prev_matched_cam0_points);
//   removeUnmarkedElements(prev_tracked_cam1_points, match_inliers, prev_matched_cam1_points);
//   removeUnmarkedElements(curr_tracked_cam0_points, match_inliers, curr_matched_cam0_points);
//   removeUnmarkedElements(curr_cam1_points, match_inliers, curr_matched_cam1_points);
// 
//   // Number of features left after stereo matching.
//   after_matching = curr_matched_cam0_points.size();
// 
//   // Step 2 and 3: RANSAC on temporal image pairs of cam0 and cam1.
//   vector<int> cam0_ransac_inliers(0);
//   twoPointRansac(prev_matched_cam0_points, curr_matched_cam0_points,
//       cam0_R_p_c, cam0_intrinsics, cam0_distortion_model,
//       cam0_distortion_coeffs, processor_config.ransac_threshold,
//       0.99, cam0_ransac_inliers);
// 
//   vector<int> cam1_ransac_inliers(0);
//   twoPointRansac(prev_matched_cam1_points, curr_matched_cam1_points,
//       cam1_R_p_c, cam1_intrinsics, cam1_distortion_model,
//       cam1_distortion_coeffs, processor_config.ransac_threshold,
//       0.99, cam1_ransac_inliers);
// 
//   // Number of features after ransac.
//   after_ransac = 0;
// 
//   for (int i = 0; i < cam0_ransac_inliers.size(); ++i) {
//     if (cam0_ransac_inliers[i] == 0 ||
//         cam1_ransac_inliers[i] == 0) continue;
//     int row = static_cast<int>(
//         curr_matched_cam0_points[i].y / grid_height);
//     int col = static_cast<int>(
//         curr_matched_cam0_points[i].x / grid_width);
//     int code = row*processor_config.grid_col + col;
//     (*curr_features_ptr)[code].push_back(FeatureMetaData());
// 
//     FeatureMetaData& grid_new_feature = (*curr_features_ptr)[code].back();
//     grid_new_feature.id = prev_matched_ids[i];
//     grid_new_feature.lifetime = ++prev_matched_lifetime[i];
//     grid_new_feature.cam0_point = curr_matched_cam0_points[i];
//     grid_new_feature.cam1_point = curr_matched_cam1_points[i];
// 
//     ++after_ransac;
//   }
// 
//   // Compute the tracking rate.
//   int prev_feature_num = 0;
//   for (const auto& item : *prev_features_ptr)
//     prev_feature_num += item.second.size();
// 
//   int curr_feature_num = 0;
//   for (const auto& item : *curr_features_ptr)
//     curr_feature_num += item.second.size();
// //设置打印周期位0.5
//   ROS_INFO_THROTTLE(0.5,
//       "\033[0;32m candidates: %d; track: %d; match: %d; ransac: %d/%d=%f\033[0m",
//       before_tracking, after_tracking, after_matching,
//       curr_feature_num, prev_feature_num,
//       static_cast<double>(curr_feature_num)/
//       (static_cast<double>(prev_feature_num)+1e-5));
//   //printf(
//   //    "\033[0;32m candidates: %d; raw track: %d; stereo match: %d; ransac: %d/%d=%f\033[0m\n",
//   //    before_tracking, after_tracking, after_matching,
//   //    curr_feature_num, prev_feature_num,
//   //    static_cast<double>(curr_feature_num)/
//   //    (static_cast<double>(prev_feature_num)+1e-5));
// 
//   return;
//  }
// 
//  void ORBmatcher::twoPointRansac( const vector<Point2f>& pts1, const vector<Point2f>& pts2,
//     const cv::Matx33f& R_p_c, const cv::Vec4d& intrinsics,
//     const std::string& distortion_model, const cv::Vec4d& distortion_coeffs,
//     const double& inlier_error, const double& success_probability, vector<int>& inlier_markers)
// {
// 
//   // Check the size of input point size.
//   if (pts1.size() != pts2.size())
//     ROS_ERROR("Sets of different size (%lu and %lu) are used...",
//         pts1.size(), pts2.size());
// 
//   double norm_pixel_unit = 2.0 / (intrinsics[0]+intrinsics[1]);
//   int iter_num = static_cast<int>(
//       ceil(log(1-success_probability) / log(1-0.7*0.7)));
// 
//   // Initially, mark all points as inliers.
//   inlier_markers.clear();
//   inlier_markers.resize(pts1.size(), 1);
// 
//   // Undistort all the points.
//   vector<cv::Point2f> pts1_undistorted(pts1.size());
//   vector<cv::Point2f> pts2_undistorted(pts2.size());
//   cv::undistortPoints( pts1, intrinsics, distortion_model, distortion_coeffs, pts1_undistorted);
//   cv::undistortPoints( pts2, intrinsics, distortion_model,  distortion_coeffs, pts2_undistorted);
// 
//   // Compenstate the points in the previous image with
//   // the relative rotation.
//   for (auto& pt : pts1_undistorted) {
//     Vec3f pt_h(pt.x, pt.y, 1.0f);
//     //Vec3f pt_hc = dR * pt_h;
//     Vec3f pt_hc = R_p_c * pt_h;
//     pt.x = pt_hc[0];
//     pt.y = pt_hc[1];
//   }
// 
//   // Normalize the points to gain numerical stability.
//   float scaling_factor = 0.0f;
//   rescalePoints(pts1_undistorted, pts2_undistorted, scaling_factor);
//   norm_pixel_unit *= scaling_factor;
// 
//   // Compute the difference between previous and current points,
//   // which will be used frequently later.
//   vector<  cv::Point2d> pts_diff(pts1_undistorted.size());
//   for (int i = 0; i < pts1_undistorted.size(); ++i)
//     pts_diff[i] = pts1_undistorted[i] - pts2_undistorted[i];
// 
//   // Mark the point pairs with large difference directly.
//   // BTW, the mean distance of the rest of the point pairs
//   // are computed.
//   double mean_pt_distance = 0.0;
//   int raw_inlier_cntr = 0;
//   for (int i = 0; i < pts_diff.size(); ++i) {
//     double distance = sqrt(pts_diff[i].dot(pts_diff[i]));
//     // 25 pixel distance is a pretty large tolerance for normal motion.
//     // However, to be used with aggressive motion, this tolerance should
//     // be increased significantly to match the usage.
//     if (distance > 50.0*norm_pixel_unit) {
//       inlier_markers[i] = 0;
//     } else {
//       mean_pt_distance += distance;
//       ++raw_inlier_cntr;
//     }
//   }
//   mean_pt_distance /= raw_inlier_cntr;
// 
//   // If the current number of inliers is less than 3, just mark
//   // all input as outliers. This case can happen with fast
//   // rotation where very few features are tracked.
//   if (raw_inlier_cntr < 3) {
//     for (auto& marker : inlier_markers) marker = 0;
//     return;
//   }
// 
//   // Before doing 2-point RANSAC, we have to check if the motion
//   // is degenerated, meaning that there is no translation between
//   // the frames, in which case, the model of the RANSAC does not
//   // work. If so, the distance between the matched points will
//   // be almost 0.
//   //if (mean_pt_distance < inlier_error*norm_pixel_unit) {
//   if (mean_pt_distance < norm_pixel_unit) {
//     //ROS_WARN_THROTTLE(1.0, "Degenerated motion...");
//     for (int i = 0; i < pts_diff.size(); ++i) {
//       if (inlier_markers[i] == 0) continue;
//       if (sqrt(pts_diff[i].dot(pts_diff[i])) >
//           inlier_error*norm_pixel_unit)
//         inlier_markers[i] = 0;
//     }
//     return;
//   }
// 
//   // In the case of general motion, the RANSAC model can be applied.
//   // The three column corresponds to tx, ty, and tz respectively.
//   MatrixXd coeff_t(pts_diff.size(), 3);
//   for (int i = 0; i < pts_diff.size(); ++i) {
//     coeff_t(i, 0) = pts_diff[i].y;
//     coeff_t(i, 1) = -pts_diff[i].x;
//     coeff_t(i, 2) = pts1_undistorted[i].x*pts2_undistorted[i].y -
//       pts1_undistorted[i].y*pts2_undistorted[i].x;
//   }
// 
//   vector<int> raw_inlier_idx;
//   for (int i = 0; i < inlier_markers.size(); ++i) {
//     if (inlier_markers[i] != 0)
//       raw_inlier_idx.push_back(i);
//   }
// 
//   vector<int> best_inlier_set;
//   double best_error = 1e10;
//   random_numbers::RandomNumberGenerator random_gen;
// 
//   for (int iter_idx = 0; iter_idx < iter_num; ++iter_idx) {
//     // Randomly select two point pairs.
//     // Although this is a weird way of selecting two pairs, but it
//     // is able to efficiently avoid selecting repetitive pairs.
//     int select_idx1 = random_gen.uniformInteger(
//         0, raw_inlier_idx.size()-1);
//     int select_idx_diff = random_gen.uniformInteger(
//         1, raw_inlier_idx.size()-1);
//     int select_idx2 = select_idx1+select_idx_diff<raw_inlier_idx.size() ?
//       select_idx1+select_idx_diff :
//       select_idx1+select_idx_diff-raw_inlier_idx.size();
// 
//     int pair_idx1 = raw_inlier_idx[select_idx1];
//     int pair_idx2 = raw_inlier_idx[select_idx2];
// 
//     // Construct the model;
//     Vector2d coeff_tx(coeff_t(pair_idx1, 0), coeff_t(pair_idx2, 0));
//     Vector2d coeff_ty(coeff_t(pair_idx1, 1), coeff_t(pair_idx2, 1));
//     Vector2d coeff_tz(coeff_t(pair_idx1, 2), coeff_t(pair_idx2, 2));
//     vector<double> coeff_l1_norm(3);
//     coeff_l1_norm[0] = coeff_tx.lpNorm<1>();
//     coeff_l1_norm[1] = coeff_ty.lpNorm<1>();
//     coeff_l1_norm[2] = coeff_tz.lpNorm<1>();
//     int base_indicator = min_element(coeff_l1_norm.begin(),
//         coeff_l1_norm.end())-coeff_l1_norm.begin();
// 
//     Vector3d model(0.0, 0.0, 0.0);
//     if (base_indicator == 0) {
//       Matrix2d A;
//       A << coeff_ty, coeff_tz;
//       Vector2d solution = A.inverse() * (-coeff_tx);
//       model(0) = 1.0;
//       model(1) = solution(0);
//       model(2) = solution(1);
//     } else if (base_indicator ==1) {
//       Matrix2d A;
//       A << coeff_tx, coeff_tz;
//       Vector2d solution = A.inverse() * (-coeff_ty);
//       model(0) = solution(0);
//       model(1) = 1.0;
//       model(2) = solution(1);
//     } else {
//       Matrix2d A;
//       A << coeff_tx, coeff_ty;
//       Vector2d solution = A.inverse() * (-coeff_tz);
//       model(0) = solution(0);
//       model(1) = solution(1);
//       model(2) = 1.0;
//     }
// 
//     // Find all the inliers among point pairs.
//     VectorXd error = coeff_t * model;
// 
//     vector<int> inlier_set;
//     for (int i = 0; i < error.rows(); ++i) {
//       if (inlier_markers[i] == 0) continue;
//       if (std::abs(error(i)) < inlier_error*norm_pixel_unit)
//         inlier_set.push_back(i);
//     }
// 
//     // If the number of inliers is small, the current
//     // model is probably wrong.
//     if (inlier_set.size() < 0.2*pts1_undistorted.size())
//       continue;
// 
//     // Refit the model using all of the possible inliers.
//     VectorXd coeff_tx_better(inlier_set.size());
//     VectorXd coeff_ty_better(inlier_set.size());
//     VectorXd coeff_tz_better(inlier_set.size());
//     for (int i = 0; i < inlier_set.size(); ++i) {
//       coeff_tx_better(i) = coeff_t(inlier_set[i], 0);
//       coeff_ty_better(i) = coeff_t(inlier_set[i], 1);
//       coeff_tz_better(i) = coeff_t(inlier_set[i], 2);
//     }
// 
//     Vector3d model_better(0.0, 0.0, 0.0);
//     if (base_indicator == 0) {
//       MatrixXd A(inlier_set.size(), 2);
//       A << coeff_ty_better, coeff_tz_better;
//       Vector2d solution =
//           (A.transpose() * A).inverse() * A.transpose() * (-coeff_tx_better);
//       model_better(0) = 1.0;
//       model_better(1) = solution(0);
//       model_better(2) = solution(1);
//     } else if (base_indicator ==1) {
//       MatrixXd A(inlier_set.size(), 2);
//       A << coeff_tx_better, coeff_tz_better;
//       Vector2d solution =
//           (A.transpose() * A).inverse() * A.transpose() * (-coeff_ty_better);
//       model_better(0) = solution(0);
//       model_better(1) = 1.0;
//       model_better(2) = solution(1);
//     } else {
//       MatrixXd A(inlier_set.size(), 2);
//       A << coeff_tx_better, coeff_ty_better;
//       Vector2d solution =
//           (A.transpose() * A).inverse() * A.transpose() * (-coeff_tz_better);
//       model_better(0) = solution(0);
//       model_better(1) = solution(1);
//       model_better(2) = 1.0;
//     }
// 
//     // Compute the error and upate the best model if possible.
//     VectorXd new_error = coeff_t * model_better;
// 
//     double this_error = 0.0;
//     for (const auto& inlier_idx : inlier_set)
//       this_error += std::abs(new_error(inlier_idx));
//     this_error /= inlier_set.size();
// 
//     if (inlier_set.size() > best_inlier_set.size()) {
//       best_error = this_error;
//       best_inlier_set = inlier_set;
//     }
//   }
// 
//   // Fill in the markers.
//   inlier_markers.clear();
//   inlier_markers.resize(pts1.size(), 0);
//   for (const auto& inlier_idx : best_inlier_set)
//     inlier_markers[inlier_idx] = 1;
// 
//   //printf("inlier ratio: %lu/%lu\n",
//   //    best_inlier_set.size(), inlier_markers.size());
// 
//   return;
// }


 /*********************************************************************************************/
 int ORBmatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint*> &sAlreadyFound, const float th , const int ORBdist)
{
    int nmatches = 0;

    const cv::Mat Rcw = CurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    const cv::Mat tcw = CurrentFrame.mTcw.rowRange(0,3).col(3);
    const cv::Mat Ow = -Rcw.t()*tcw;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMPs[i];

        if(pMP)
        {
            if(!pMP->isBad() && !sAlreadyFound.count(pMP))
            {
                //Project
                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw*x3Dw+tcw;

                const float xc = x3Dc.at<float>(0);
                const float yc = x3Dc.at<float>(1);
                const float invzc = 1.0/x3Dc.at<float>(2);

                const float u = CurrentFrame.fx*xc*invzc+CurrentFrame.cx;
                const float v = CurrentFrame.fy*yc*invzc+CurrentFrame.cy;

                if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
                    continue;
                if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
                    continue;

                // Compute predicted scale level
                cv::Mat PO = x3Dw-Ow;
                float dist3D = cv::norm(PO);

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();

                // Depth must be inside the scale pyramid of the image
                if(dist3D<minDistance || dist3D>maxDistance)
                    continue;

                int nPredictedLevel = pMP->PredictScale(dist3D,&CurrentFrame);

                // Search in a window
                const float radius = th*CurrentFrame.mvScaleFactors[nPredictedLevel];

                const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(u, v, radius, nPredictedLevel-1, nPredictedLevel+1);

                if(vIndices2.empty())
                    continue;

                const cv::Mat dMP = pMP->GetDescriptor();

                int bestDist = 256;
                int bestIdx2 = -1;

                for(vector<size_t>::const_iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
                {
                    const size_t i2 = *vit;
                    if(CurrentFrame.mvpMapPoints[i2])
                        continue;

                    const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

                    const int dist = DescriptorDistance(dMP,d);

                    if(dist<bestDist)
                    {
                        bestDist=dist;
                        bestIdx2=i2;
                    }
                }

                if(bestDist<=ORBdist)
                {
                    CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
                    nmatches++;

                    if(mbCheckOrientation)
                    {
                        float rot = pKF->mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                        if(rot<0.0)
                            rot+=360.0f;
                        int bin = round(rot*factor);
                        if(bin==HISTO_LENGTH)
                            bin=0;
                        assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin].push_back(bestIdx2);
                    }
                }

            }
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i!=ind1 && i!=ind2 && i!=ind3)
            {
                for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]]=NULL;
                    nmatches--;
                }
            }
        }
    }

    return nmatches;
}

void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

} //namespace ORB_SLAM
