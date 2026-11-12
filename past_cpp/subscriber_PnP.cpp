#include "rclcpp/rclcpp.hpp"
#include "rm_msgs/msg/detection_array.hpp"
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class VisionSubscriber : public rclcpp::Node
{
public:
    VisionSubscriber() : Node("vision_subscriber")
    {
        subscription_ = this->create_subscription<rm_msgs::msg::DetectionArray>(
            "/vision", 10, std::bind(&VisionSubscriber::vision_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "订阅节点已启动");
    }

private:
    void vision_callback(const rm_msgs::msg::DetectionArray::SharedPtr msg) const
    {
        if (msg->detections.empty()) return;//return：退出函数

        auto d = msg->detections[0];

        // 图像点：4个角点像素坐标（bag顺序：左上→左下→右下→右上）
        std::vector<cv::Point2f> image_points = {//cv::Point2f表示2D点的坐标
            {(float)d.x1, (float)d.y1},
            {(float)d.x2, (float)d.y2},
            {(float)d.x3, (float)d.y3},
            {(float)d.x4, (float)d.y4}
        };

        // 物点：4个角在板子坐标系的3D坐标（米），顺序与图像点一一对应——物点用什么单位，pnpsolve就吐什么单位
        const double w = 0.135;   // 小装甲板宽135mm，统一成m为单位
        const double h = 0.054;   // 高54mm
        std::vector<cv::Point3f> object_points = {//物点：以板子中心为原点，角点的坐标
            {-(float)w/2, -(float)h/2, 0.0f},   // 左上
            {-(float)w/2,  (float)h/2, 0.0f},   // 左下
            { (float)w/2,  (float)h/2, 0.0f},   // 右下
            { (float)w/2, -(float)h/2, 0.0f}    // 右上
        };

        // 相机内参：camera_info → OpenCV矩阵——cv：：Mat
        cv::Mat K = cv::Mat(3, 3, CV_64F);//创建3*3矩阵，填入64位浮点数
        for (int i = 0; i < 9; i++)
            K.at<double>(i / 3, i % 3) = msg->camera_info.k[i];//camera_info是1*9向量，i/3取商，i%3取余数
        cv::Mat dist = cv::Mat(1, 5, CV_64F);
        for (int i = 0; i < 5; i++)
            dist.at<double>(0, i) = msg->camera_info.d[i];

        // PnP解算
        cv::Vec3d rvec, tvec;//创建两个对象
        bool ok = cv::solvePnP(object_points, image_points, K, dist,
                               rvec, tvec, false, cv::SOLVEPNP_IPPE);//false：不用初始猜测；solvePnP中有多种求解算法，用IPPE——用于共面4点

        if (ok) {
            RCLCPP_INFO(this->get_logger(),
                "装甲板位置(相机系): x=%.3f  y=%.3f  z=%.3f (米)",
                tvec[0], tvec[1], tvec[2]);
        }
    }

    rclcpp::Subscription<rm_msgs::msg::DetectionArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionSubscriber>());
    rclcpp::shutdown();
    return 0;
}