#ifndef YOLOP_NODE_H
#define YOLOP_NODE_H

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/opencv.hpp>

#include "yolop.hpp"

class YOLOPNode {
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;

    ros::Subscriber img_sub;
    image_transport::Publisher res_pub;

    bool use_morai;

    YOLOP yolop;

public:
    YOLOPNode();
    ~YOLOPNode();

    void imgCallback(const sensor_msgs::ImageConstPtr& msg);
    void compressedImgCallback(const sensor_msgs::CompressedImageConstPtr& msg);

    void publishYOLOPresult(const cv::Mat& img);
};

#endif
