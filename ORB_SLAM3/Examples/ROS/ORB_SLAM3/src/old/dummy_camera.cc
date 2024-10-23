#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>
#include <ros/ros.h>
#include <sstream>
#include "std_msgs/Float64MultiArray.h"
#include<opencv2/core/core.hpp>
#include"../../../include/System.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>



using namespace std;


void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("dummy_camera/image", 1);

vector<string> vstrImageFilenames;
vector<double> vTimestamps;
string strFile = "/home/orbrostest/SLAM/Dataset/rgb.txt";
LoadImages(strFile, vstrImageFilenames, vTimestamps);
int nImages = vstrImageFilenames.size();


  ros::Rate loop_rate(5);
  while (nh.ok()) {

    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread("/home/orbrostest/SLAM/Dataset/"+vstrImageFilenames[ni],cv::IMREAD_UNCHANGED);
        cv::imshow("sfdlpojfs", im);
        cv::waitKey(5);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
            return 1;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im).toImageMsg();

        pub.publish(msg);
    }



  }
}



