#include "rclcpp/rclcpp.hpp"
#include "rm_msgs/msg/detection_array.hpp"
#include <opencv2/opencv.hpp>
#include "tf2_ros/buffer.h"                          // tf 台账
#include "tf2_ros/transform_listener.h"              // tf 监听器
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"   // 让 tf 能变换 Point 类型
#include "geometry_msgs/msg/point_stamped.hpp"       // 带坐标系标签的点
#include "visualization_msgs/msg/marker_array.hpp"   // rviz画球的消息类型
#include <vector>                                    // std::vector 的家
#include <cmath>                                     // std::sqrt 的家
using std::placeholders::_1;

#include "armor_tracker/kalman_filter.hpp"

// ==================== 追踪器 = 滤波器 + 档案信息 ====================
// struct：所有成员默认公开的简易class，适合纯装数据
struct Tracker {
    Tracker(double q_pos, double q_vel, double r) : kf(q_pos, q_vel, r) {}//成员初始化列表——用tracker构造时收到的三个参数去构造成员kf
    KalmanFilter kf;     // 这个目标的滤波器
    int id;              // 编号（创建时分配，终身不变）
    int lost = 0;        // 连续多少帧没见到这个目标
    bool fresh = false;  // 本帧是否已经匹配到检测（防一帧内重复匹配）
    double last_z = 0;   // 最近一次的高度（画预测球用）
    int vote_candidate = -1;    // 摩尔投票：当前领先的身份（-1 = 还没投过票）
    double vote_count = 0;      // 摩尔投票：领先票数（带score权重）

};

// ==================== 订阅节点 ====================
class VisionSubscriber : public rclcpp::Node {
public:
    VisionSubscriber() : Node("vision_subscriber") {
        {
            // 向ros注册三个参数，注册后，节点会出现在参数服务器，用ros2 param系列命令就能调参
            this->declare_parameter<double>("q_pos", 0.01);
            this->declare_parameter<double>("q_vel", 0.5);
            this->declare_parameter<double>("r", 0.05);
            this->declare_parameter<double>("pred_dt",0.1);

            // 参数被修改时的回调：把所有追踪器的 Q、R 换成新值
            param_callback_ = this->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter>& params) {
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;//是否接受此次修改，可在这里添加if
                    for (const auto& p : params) {
                        if (p.get_name() == "q_pos") q_pos_ = p.as_double();
                        if (p.get_name() == "q_vel") q_vel_ = p.as_double();
                        if (p.get_name() == "r") r_ = p.as_double();
                        if (p.get_name()=="pred_dt") pred_dt_=p.as_double();
                    }
                    for (auto& t : trackers_) {
                        t.kf.setQ(q_pos_, q_vel_);
                        t.kf.setR(r_);
                    }
                    RCLCPP_INFO(this->get_logger(), "参数已更新: q_pos=%.3f q_vel=%.3f r=%.4f", q_pos_, q_vel_, r_);
                    return result;
                });

            // ...原来的 subscription_ / tf_buffer_ / marker_pub_ 初始化一行不动...
        }
        subscription_ = this->create_subscription<rm_msgs::msg::DetectionArray>(
            "/vision", 10, std::bind(&VisionSubscriber::vision_callback, this, _1));
        // 创建 tf 台账 + 监听器（监听器自动在后台收 /tf，往台账里存）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());       // 向节点对象要时钟
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // 写到台账
        // 创建画球发布者：往 /armor_markers 话题发 Marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/armor_markers", 10);
        RCLCPP_INFO(this->get_logger(), "订阅节点已启动");
    }
private:
    //造一个球的辅助函数：位置、编号、颜色由参数决定
    visualization_msgs::msg::Marker make_ball(double x, double y, double z,
                                              int id, float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "gimbal_link";          // 标签：球画在云台系
        m.header.stamp = this->get_clock()->now();  // 时间戳：现在
        m.ns = "armor";                             // 命名空间：分组标签
        m.id = id;                                  // 球的编号：同ns下靠id区分，同id被覆盖更新
        m.type = visualization_msgs::msg::Marker::SPHERE;  // 形状：球
        m.action = visualization_msgs::msg::Marker::ADD;   // 动作：添加/更新
        m.pose.position.x = x;                      // 球心位置
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.scale.x = m.scale.y = m.scale.z = 0.1;   
        m.color.r = r;  m.color.g = g;  m.color.b = b;  // 颜色（0~1）
        m.color.a = 1.0;                            // 透明度：1.0 = 不透明
        m.lifetime = rclcpp::Duration::from_seconds(0.01);//0.01秒没更新自动消失（防残影）
        return m;
    }

    void vision_callback(const rm_msgs::msg::DetectionArray::SharedPtr msg) {
        double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;  // 这一帧的时间戳

        //  所有追踪器先猜：各自的目标现在应该在哪（空帧也要推进时钟）
        for (auto &t : trackers_) t.kf.predict(stamp);

        visualization_msgs::msg::MarkerArray arr;   // 这一帧要画的所有球，打包
        int ball_id = 0;                            // 球的编号分配器

        //  处理这一帧的【每一个】检测：逐块装甲板走完整流水线
        for (auto &d : msg->detections) {

            // ---- PnP解算：2D角点 → 相机系3D位置 ----
            std::vector<cv::Point2f> image_points = {
                {(float)d.x1, (float)d.y1}, {(float)d.x2, (float)d.y2},
                {(float)d.x3, (float)d.y3}, {(float)d.x4, (float)d.y4}
            };
            const double w = 0.135; const double h = 0.054;   // 小装甲板尺寸（米）
            std::vector<cv::Point3f> object_points = {
                {-(float)w/2, -(float)h/2, 0.0f}, {-(float)w/2, (float)h/2, 0.0f},
                {(float)w/2, (float)h/2, 0.0f}, {(float)w/2, -(float)h/2, 0.0f}
            };
            cv::Mat K = cv::Mat(3, 3, CV_64F);
            for (int i = 0; i < 9; i++) K.at<double>(i/3, i%3) = msg->camera_info.k[i];
            cv::Mat dist = cv::Mat(1, 5, CV_64F);
            for (int i = 0; i < 5; i++) dist.at<double>(0, i) = msg->camera_info.d[i];
            cv::Vec3d rvec, tvec;
            bool ok = cv::solvePnP(object_points, image_points, K, dist, rvec, tvec,
                                   false, cv::SOLVEPNP_IPPE);   // 可能无解
            if (!ok) continue;              // 这块板解算失败：跳过它，处理下一块

            // ===== tf 坐标变换 =====
            // 1. 把 PnP 算出的点包装成"带标签的点"：告诉 tf 它在哪个坐标系
            geometry_msgs::msg::PointStamped p_cam;
            p_cam.header.frame_id = "camera_optical_frame";   // 标签：我来自相机系
            p_cam.point.x = tvec[0];
            p_cam.point.y = tvec[1];
            p_cam.point.z = tvec[2];
            // 2. 查台账，变换到 gimbal_link
            geometry_msgs::msg::PointStamped p_gimbal;   // 先声明在try外面，后面还要用
            try {
                p_gimbal = tf_buffer_->transform(p_cam, "gimbal_link",
                                                 tf2::durationFromSec(0.1));
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "tf查询失败: %s", ex.what());
                continue;                   // 这块板变换失败：跳过它，处理下一块
            }
            double mx = p_gimbal.point.x, my = p_gimbal.point.y;   // 观测位置

            // ---- 帧间匹配：找离测量点最近的追踪器 ----
            const double match_thresh = 0.45;    // 阈值 = 测量噪声幅度 + 机动余量
            int best = -1;                       // 最近追踪器的编号，-1 = 还没找到
            double best_dist = 1e9;              // 最近距离，先设成天文数字
            for (int i = 0; i < (int)trackers_.size(); i++) {
                if (trackers_[i].fresh) continue;          // 本帧已被别的检测认领
                double dx = mx - trackers_[i].kf.get_x();  // 测量和猜测的横向差距
                double dy = my - trackers_[i].kf.get_y();  // 纵向差距
                double dd = std::sqrt(dx*dx + dy*dy);      // 欧氏距离 = √(dx²+dy²)
                if (dd < best_dist) { best_dist = dd; best = i; }  // 刷新最近纪录
            }

            // ---- 并入旧实例 或 创建新实例 ----
            if (best == -1 || best_dist > match_thresh) {
                trackers_.push_back(Tracker(q_pos_,q_vel_,r_));               // 放进数组
                auto & tr = trackers_.back();//那引用指向刚刚放进去的那个
                tr.kf.init(mx, my, stamp);             // 第一次见到目标：初始化
                tr.id = next_id_++;                    // 分配编号：先用后加
                best = (int)trackers_.size() - 1;
                RCLCPP_WARN(this->get_logger(), "新目标出现，创建追踪器 #%d", tr.id);
            } else {
                trackers_[best].kf.update(mx, my);     // 同一目标：正常融合
            }
            trackers_[best].fresh = true;              // 标记：本帧已见到
            trackers_[best].lost = 0;                  // 连续丢失计数清零
            trackers_[best].last_z = p_gimbal.point.z; // 记住高度（画球用）

            // ---- 摩尔投票：score加权，投出稳定身份 ----
            int identity = d.color * 10 + d.type;   // 颜色×10+型号 = 身份码
            double weight = d.score;                // 置信度当票数
            if (trackers_[best].vote_candidate == identity) {
                trackers_[best].vote_count += weight;         // 同身份：加票
            } else {
                trackers_[best].vote_count -= weight;         // 不同身份：抵消
                if (trackers_[best].vote_count <= 0) {        // 旧候选被磨光了：换候选
                    trackers_[best].vote_candidate = identity;
                    trackers_[best].vote_count = weight;
                }
            }

            // 观测球（绿色）：画在测量位置
            arr.markers.push_back(make_ball(mx, my, p_gimbal.point.z,
                                            ball_id++, 0.0, 1.0, 0.0));
        }

        //  生命周期管理：本帧没匹配到的追踪器 lost++，超时删除
        const int max_lost = 10;                       // 阈值t：连续10帧没见到就放弃它
        for (int i = (int)trackers_.size() - 1; i >= 0; i--) {  // 倒序遍历，删除不影响前面
            if (!trackers_[i].fresh) trackers_[i].lost++;
            if (trackers_[i].lost > max_lost) {
                RCLCPP_WARN(this->get_logger(),
                    "追踪器 #%d 连续%d帧未见，删除", trackers_[i].id, max_lost);
                trackers_.erase(trackers_.begin() + i);  // 从数组里删除
            }
        }

        //  打印 + 预测球（红）+ 滤波球（黄）：只有本帧匹配到检测的追踪器才画球（日志照打）
        for (auto &t : trackers_) {
            double px, py;
            t.kf.predict_pos(pred_dt_, px, py);             // 往前猜0.1秒
            RCLCPP_INFO(this->get_logger(),
                "[#%d] 滤波: (%.3f, %.3f)  预测0.1s: (%.3f, %.3f)  身份码: %d (票数 %.1f)",
                t.id, t.kf.get_x(), t.kf.get_y(), px, py, t.vote_candidate, t.vote_count);
            if (t.fresh) {                                  // 本帧没见到这个目标：不画它的球
                // 预测球（红色）：画在0.1秒后的位置
                arr.markers.push_back(make_ball(px, py, t.last_z,
                                                ball_id++, 1.0, 0.0, 0.0));
                // 滤波球（黄色）：画在本帧卡尔曼滤波后的位置，应与绿色观测球基本重合
                arr.markers.push_back(make_ball(t.kf.get_x(), t.kf.get_y(), t.last_z,
                                                ball_id++, 1.0, 1.0, 0.0));
            }
            t.fresh = false;                           // 清零旗标，为下一帧做准备
        }

        if (!arr.markers.empty()) marker_pub_->publish(arr);   // 一整帧的球打包发出
    }

    rclcpp::Subscription<rm_msgs::msg::DetectionArray>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;              // 台账
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // 监听器
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;  // 画球发布者
    std::vector<Tracker> trackers_;   // 所有活跃目标的追踪器（每个目标一个kf对象+档案）
    int next_id_ = 0;                 // 编号分配器：下一个新目标用几号
    double q_pos_=0.01,q_vel_=0.5,r_=0.05;
    OnSetParametersCallbackHandle::SharedPtr param_callback_;
    double pred_dt_=0.1;
};
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionSubscriber>());
    rclcpp::shutdown();
    return 0;
}