#ifndef ARMOR_TRACKER__KALMAN_FILTER_HPP_
#define ARMOR_TRACKER__KALMAN_FILTER_HPP_

#include <opencv2/opencv.hpp>

class KalmanFilter
{
public:
  // q_pos: 位置过程噪声  q_vel: 速度过程噪声  r: 量测噪声
  KalmanFilter(double q_pos, double q_vel, double r)
  : q_pos_(q_pos), q_vel_(q_vel), r_(r)
  {
    x_ = cv::Mat::zeros(4, 1, CV_64F);
    buildQR();
    P_ = cv::Mat::eye(4, 4, CV_64F) * 0.1;
  }

  void setQ(double q_pos, double q_vel)   // 动态调参入口
  {
    q_pos_ = q_pos;
    q_vel_ = q_vel;
    buildQR();
  }

  void setR(double r)                     // 动态调参入口
  {
    r_ = r;
    buildQR();
  }

  void init(double x, double y, double stamp)
  {
    x_.at<double>(0) = x;
    x_.at<double>(1) = y;
    x_.at<double>(2) = 0;
    x_.at<double>(3) = 0;
    P_ = cv::Mat::eye(4, 4, CV_64F) * 0.1;
    last_stamp_ = stamp;
    initialized_ = true;
  }

  bool is_initialized() const { return initialized_; }

  void predict(double stamp)
  {
    double dt = stamp - last_stamp_;
    last_stamp_ = stamp;
    cv::Mat F = (cv::Mat_<double>(4, 4) <<
      1, 0, dt, 0,
      0, 1, 0, dt,
      0, 0, 1, 0,
      0, 0, 0, 1);
    x_ = F * x_;
    P_ = F * P_ * F.t() + Q_;
  }

  void update(double z_x, double z_y)
  {
    cv::Mat H = (cv::Mat_<double>(2, 4) <<
      1, 0, 0, 0,
      0, 1, 0, 0);
    cv::Mat z = (cv::Mat_<double>(2, 1) << z_x, z_y);
    cv::Mat y = z - H * x_;
    cv::Mat S = H * P_ * H.t() + R_;
    cv::Mat K = P_ * H.t() * S.inv();
    x_ = x_ + K * y;
    P_ = (cv::Mat::eye(4, 4, CV_64F) - K * H) * P_;
  }

  void predict_pos(double dt_ahead, double & px, double & py) const
  {
    px = x_.at<double>(0) + x_.at<double>(2) * dt_ahead;
    py = x_.at<double>(1) + x_.at<double>(3) * dt_ahead;
  }

  double get_x() const { return x_.at<double>(0); }
  double get_y() const { return x_.at<double>(1); }
  double get_vx() const { return x_.at<double>(2); }
  double get_vy() const { return x_.at<double>(3); }

private:
  void buildQR()   // Q、R 由参数生成：位置/速度解耦
  {
    Q_ = (cv::Mat_<double>(4, 4) <<
      q_pos_, 0, 0, 0,
      0, q_pos_, 0, 0,
      0, 0, q_vel_, 0,
      0, 0, 0, q_vel_);
    R_ = (cv::Mat_<double>(2, 2) <<
      r_, 0,
      0, r_);
  }

  cv::Mat x_, P_, Q_, R_;
  double q_pos_, q_vel_, r_;
  double last_stamp_ = 0;
  bool initialized_ = false;
};

#endif  // ARMOR_TRACKER__KALMAN_FILTER_HPP_