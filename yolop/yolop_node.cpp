#include "yolop_node.h"

YOLOPNode::YOLOPNode(): it(nh){
    const std::string& pkg_path = ros::package::getPath("vision");

    this->yolop.read_engine_file(pkg_path + "/src/tensorrtx/yolop/build/yolop.trt");

    this->res_pub = it.advertise("/lane_segment", 1);

    ros::param::get("~use_morai", this->use_morai);

    if(use_morai) {
        this->img_sub = nh.subscribe("/image_jpeg/compressed", 1, &YOLOPNode::compressedImgCallback, this);
    }
    
    else {
        this->img_sub = nh.subscribe("/usb_cam/image_raw", 1, &YOLOPNode::imgCallback, this);
    }
}

YOLOPNode::~YOLOPNode(){
    cv::destroyAllWindows();
}

void YOLOPNode::imgCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    this->publishYOLOPresult(img);
}

void YOLOPNode::compressedImgCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    this->publishYOLOPresult(img);
}

void YOLOPNode::publishYOLOPresult(const cv::Mat& img){
    this->yolop.set_img(img);
    auto inf = this->yolop.inference();
    this->yolop.visualization(inf);

    cv_bridge::CvImage bridge(
        std_msgs::Header(), sensor_msgs::image_encodings::BGR8, this->yolop.get_img());

    this->res_pub.publish(bridge.toImageMsg());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolop_node");
    
    YOLOPNode node;
    ros::spin();

    return 0;
}
