/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float64MultiArray.h"
#include<opencv2/core/core.hpp>
#include <sstream>
#include"../../../include/System.h"

using namespace std;

void getQuaternion(cv::Mat R, float (&Q)[4])
{
    float trace = R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2);
 
    if (trace > 0.0) 
    {
        float s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<float>(2,1) - R.at<float>(1,2)) * s);
        Q[1] = ((R.at<float>(0,2) - R.at<float>(2,0)) * s);
        Q[2] = ((R.at<float>(1,0) - R.at<float>(0,1)) * s);
    } 
    
    else 
    {
        int i = R.at<float>(0,0) < R.at<float>(1,1) ? (R.at<float>(1,1) < R.at<float>(2,2) ? 2 : 1) : (R.at<float>(0,0) < R.at<float>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        float s = sqrt(R.at<float>(i, i) - R.at<float>(j,j) - R.at<float>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<float>(k,j) - R.at<float>(j,k)) * s;
        Q[j] = (R.at<float>(j,i) + R.at<float>(i,j)) * s;
        Q[k] = (R.at<float>(k,i) + R.at<float>(i,k)) * s;
    }
}

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};
ros::Publisher chatter_pub;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    chatter_pub = nodeHandler.advertise<std_msgs::Float64MultiArray>("camera_pose", 1000);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
	
	cv::Mat unreal_rotation = (cv::Mat_<float>(3, 3) << 0, 0, 1,
                                                        -1, 0, 0,
                                                        0, -1, 0);

	cv::Mat Tcw = cv::Mat::zeros(4, 4, CV_64F);
	Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());


	if( Tcw.empty())
	{
		
	}

	else
	{
            // unreal is zyx eueler angles system ya 5wl
            cv::Mat unreal_rotation = (cv::Mat_<float>(3, 3) << 0,  0, 1,
                                                               -1,  0, 0,
                                                                0, -1, 0);

            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            cv::Mat twcU = unreal_rotation * twc;
            cv::Mat Rot_Final = unreal_rotation * Rwc;
            float Q[4];
            getQuaternion(Rwc, Q);

            cout<<"===================opencv====================="<<endl;
            cout<<Rwc<<endl;
            cout<<"===================unreal====================="<<endl;
            cout<<Rot_Final<<endl;

            std_msgs::Float64MultiArray tezi;
            tezi.data.resize(7);

            tezi.data[0] = twcU.at<float>(0,0);
            tezi.data[1] = twcU.at<float>(0,1);
            tezi.data[2] = twcU.at<float>(0,2);
            tezi.data[3] = Q[0];
            tezi.data[4] = Q[1];
            tezi.data[5] = Q[2];
            tezi.data[6] = Q[3];
            chatter_pub.publish(tezi);

	
	}
    
}



