#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "trace/trace.h"
#include "std_msgs/Float32.h"

ros::Publisher mid_pub;
float pre_mid;

// 回调函数，处理接收到的图像消息
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // 在这里处理接收到的图像消息，可以进行图像处理、显示等操作
    pre_mid = trace_main(msg->data.data()); //进行寻迹

    // 创建浮点数消息对象
    std_msgs::Float32 float_msg;
    float_msg.data = pre_mid;

    // 发布前瞻信息
    mid_pub.publish(float_msg);
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "image_subscriber_cpp");

    // 创建 ROS 句柄
    ros::NodeHandle nh;

    // 创建订阅对象，订阅 image_topic 话题，指定回调函数
    ros::Subscriber image_sub = nh.subscribe("image_topic", 10, imageCallback);

    // 创建发布者对象，发布前瞻消息到 mid_topic 话题
    mid_pub = nh.advertise<std_msgs::Float32>("mid_topic", 1);

    // 进入 ROS 循环，等待接收消息
    ros::spin();

    return 0;
}