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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>



#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
	ros::NodeHandle nh;
	ros::Publisher  pub_rgb,pub_depth,pub_tcw,pub_camerapath;
	size_t mcounter=0;	 
	nav_msgs::Path  camerapath;
	
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM),nh("~")
	{
	   //创建ROS的发布节点

	   pub_rgb= nh.advertise<sensor_msgs::Image> ("Left/Image", 10); 
	   pub_depth= nh.advertise<sensor_msgs::Image> ("Right/Image", 10); 
	   pub_tcw= nh.advertise<geometry_msgs::PoseStamped> ("CameraPose", 10); 
	   pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 10); 
	}

    void GrabStereo(const sensor_msgs::Image::ConstPtr msgLeft,const sensor_msgs::Image::ConstPtr msgRight);
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "STEREO");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/indemind/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/indemind/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::Image::ConstPtr msgLeft,const sensor_msgs::Image::ConstPtr msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    
	bool  isKeyFrame =true;
	cv::Mat Tcw;
	
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw=mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec(),isKeyFrame);
// 		cv::imshow("left",imLeft);
// 		cv::imshow("right",imRight);
// 		cv::waitKey(1);
    }
    else
    {
        Tcw=mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec(),isKeyFrame);
    }
     
    if (!Tcw.empty())
	{
				  //cv::Mat Twc =Tcw.inv();
				  //cv::Mat TWC=orbslam->mpTracker->mCurrentFrame.mTcw.inv();  
				  cv::Mat RWC= Tcw.rowRange(0,3).colRange(0,3);  
				  cv::Mat tWC= Tcw.rowRange(0,3).col(3);

				  tf::Matrix3x3 M(RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
							      RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
							      RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2));
				  tf::Vector3 V(tWC.at<float>(0), tWC.at<float>(1), tWC.at<float>(2));
				  
				 tf::Quaternion q;
				  M.getRotation(q);
				  
			      tf::Pose tf_pose(q,V);
				  
				   double roll,pitch,yaw;
				   M.getRPY(roll,pitch,yaw);
				   //cout<<"roll: "<<roll<<"  pitch: "<<pitch<<"  yaw: "<<yaw;
				  // cout<<"    t: "<<tWC.at<float>(0)<<"   "<<tWC.at<float>(1)<<"    "<<tWC.at<float>(2)<<endl;
				   
				   if(roll == 0 || pitch==0 || yaw==0)
					return ;
				   // ------
				  
				  std_msgs::Header header ;
				  header.stamp =msgLeft->header.stamp;
				  header.seq = msgLeft->header.seq;
				  header.frame_id="camera";
				  //cout<<"depth type: "<< depth. type()<<endl;
				  
				  sensor_msgs::Image::ConstPtr rgb_msg = msgLeft;
				  sensor_msgs::Image::ConstPtr depth_msg=msgRight;
				  
				 geometry_msgs::PoseStamped tcw_msg;
				 tcw_msg.header=header;
				 tf::poseTFToMsg(tf_pose, tcw_msg.pose);
				  
				 camerapath.header =header;
				 camerapath.poses.push_back(tcw_msg);
				  
				 
				 pub_camerapath.publish(camerapath);  //相机轨迹
				 if( isKeyFrame)
				{
					pub_tcw.publish(tcw_msg);	                      //Tcw位姿信息
					pub_rgb.publish(rgb_msg);
					pub_depth.publish(depth_msg);
				}
	}
	else
	{
	  cout<<"Twc is empty ..."<<endl;
	}

}


