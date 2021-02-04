#include "ros/ros.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// File reading specific includes
#include <sys/stat.h>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;
cv::Mat imageRGB;
cv::Mat imageDepth;
sensor_msgs::PointCloud2 cloud;

size_t frameNo = 0;


bool checkIfFileExists(const std::string& name)
{
    struct stat buffer;
    return (stat (name.c_str(), &buffer)==0);
}


void readFrameNo(const std::string& name)
{
    string line;
    ifstream myFile(name);

    if (myFile.is_open())
    {
        while(getline(myFile, line))
        {
            frameNo = std::stoi(line);
        }
        myFile.close();
    }
}


void saveFrameNo(const std::string& name)
{
    ofstream myFile(name);
    if (myFile.is_open())
    {
        myFile << frameNo;
        myFile.close();
    }
}


void imageRGBCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imageRGB = cv_bridge::toCvShare(msg, "bgr8")->image;

    imshow( "RGB image", imageRGB );

    char c = waitKey(8);

    if (c=='s'){
        std::stringstream ssRGB;
        ssRGB << "rgb_" << std::setw(5) << std::setfill('0') << frameNo << ".png";
        std::string sRGB = ssRGB.str();
        imwrite(sRGB, imageRGB);

        std::stringstream ssDepth;
        ssDepth << "depth_" << std::setw(5) << std::setfill('0') << frameNo << ".png";
        std::string sDepth = ssDepth.str();
        imwrite(sDepth, imageDepth);

        std::stringstream ssCloud;
        ssCloud << "cloud_" << std::setw(5) << std::setfill('0') << frameNo << ".pcd";
        pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
        pcl::fromROSMsg(cloud, pclCloud);
        pcl::io::savePCDFile(ssCloud.str(), pclCloud);

        std::cout << "frame no " << frameNo << "\n";
        frameNo++;

        std::cout << checkIfFileExists("frameNo.txt") << std::endl;
    }
}

void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    imageDepth = cv_bridge::toCvShare(msg, "32FC1")->image;

    imageDepth.convertTo(imageDepth,CV_16UC1, 1000);

    imshow( "Depth image", imageDepth );

    waitKey(3);
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
    cloud = *inputCloud;
}

int main(int argc, char **argv)
{
    //initialize node
    ros::init(argc, argv, "cv_example");

    bool frameNoExists = checkIfFileExists("frameNo.txt");
    if (frameNoExists)
        readFrameNo("frameNo.txt");

    // node handler
    ros::NodeHandle n;

    // subsribe topic (RGB image)
    ros::Subscriber subRGB = n.subscribe("/camera/rgb/image_raw", 1000, imageRGBCallback);

    // subsribe topic (Detph image)
    ros::Subscriber subDepth = n.subscribe("/camera/depth_registered/image_raw", 1000, imageDepthCallback);

    // subsribe topic (PointCloud)
    ros::Subscriber subCloud = n.subscribe ("/camera/depth_registered/points", 100, pointCloudCallback);

    ros::spin();

    saveFrameNo("frameNo.txt");

    return 0;
}
