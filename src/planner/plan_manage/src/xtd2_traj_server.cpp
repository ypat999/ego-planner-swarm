#include "bspline_opt/uniform_bspline.h"
#include "traj_utils/msg/bspline.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_traj_pub;

geometry_msgs::msg::PoseStamped raw_cmd;

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_ = 0.5;
double time_finish_thresh_percent_ = 0.2;

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  // parse _ traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }

  // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

// 计算当前时刻的期望偏航角及偏航角速度
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;                 // 圆周率
  constexpr double YAW_DOT_MAX_PER_SEC = PI;         // 最大偏航角速度（rad/s）
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI; // 最大偏航角加速度（未使用）
  std::pair<double, double> yaw_yawdot(0, 0);      // 返回的偏航角与偏航角速度
  double yaw = 0;                                    // 当前偏航角
  double yawdot = 0;                                 // 当前偏航角速度

  // 计算前瞻方向向量：若未超出轨迹时长，则取前瞻点；否则取终点
  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                          ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos
                          : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  // 若方向向量足够长，则计算目标偏航角；否则沿用上一时刻偏航角
  double yaw_temp = dir.norm() > 0.001 ? atan2(dir(1), dir(0)) : last_yaw_;
  // 根据时间差计算本周期允许的最大偏航角变化量
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).seconds();

  // 处理跨越 ±PI 的跳变，确保角度平滑过渡
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI) yaw += 2 * PI;  // 归一化到 [-PI, PI]
      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;  // 反向旋转最快
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI) yaw -= 2 * PI;  // 归一化到 [-PI, PI]
      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;  // 正向旋转最快
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }
  else
  {
    // 无跨越 ±PI 跳变，直接限制最大角速度
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI) yaw += 2 * PI;
      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI) yaw -= 2 * PI;
      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).seconds();
    }
  }

  // 简单低通滤波，使角度与角速度更平滑
  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw;  // 朴素 LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}
void cmdCallback()
{
  /* no publishing before receive traj_ */
  if (!receive_traj_)
    return;

  // 统一时间源
  rclcpp::Clock clock(RCL_ROS_TIME);  
  rclcpp::Time time_now = clock.now();
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos_enu(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = clock.now();
  if (t_cur < traj_duration_ * (1.0 - time_finish_thresh_percent_) && t_cur >= 0.0)
  {
    pos_enu = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos_enu, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + time_forward_);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_ * (1.0 - time_finish_thresh_percent_)   )
  {
    /* hover when finish traj_ */
    pos_enu = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos_enu;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  // 将偏航角转换为四元数
  // 在ENU坐标系下构造yaw四元数（绕Z轴）
  Eigen::Quaterniond q_enu = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_yawdot.first, Eigen::Vector3d::UnitZ()));

  // 设置原始Gazebo坐标系下的位置和姿态
  raw_cmd.header.stamp = time_now;
  raw_cmd.header.frame_id = "world";
  
  raw_cmd.pose.position.x = pos_enu(0);  // ENU X
  raw_cmd.pose.position.y = pos_enu(1);  // ENU Y
  raw_cmd.pose.position.z = pos_enu(2);  // ENU Z
  
  raw_cmd.pose.orientation.x = q_enu.x();
  raw_cmd.pose.orientation.y = q_enu.y();
  raw_cmd.pose.orientation.z = q_enu.z();
  raw_cmd.pose.orientation.w = q_enu.w();

  last_yaw_ = yaw_yawdot.first;

  raw_traj_pub->publish(raw_cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");

  // Get ROS namespace parameter
  std::string ros_ns;
  node->declare_parameter("ros_ns", "");
  node->get_parameter("ros_ns", ros_ns);

  //Get time_forward parameter
  node->declare_parameter("traj_server/time_forward", 0.5);
  node->get_parameter("traj_server/time_forward", time_forward_);
  
  // Get time finish threshold parameter
  node->declare_parameter("traj_server/time_finish_thresh_percent",0.1);
  node->get_parameter("traj_server/time_finish_thresh_percent", time_finish_thresh_percent_);
  
  if (ros_ns.empty()) {
    RCLCPP_WARN(node->get_logger(), "ROS namespace not specified, using default topics");
    ros_ns = "";
  }

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  // Publish raw trajectory in Gazebo coordinates
  raw_traj_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/xtdrone2/planning/raw_trajectory",
      50);

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(50),
      cmdCallback);

  RCLCPP_INFO(node->get_logger(), "Trajectory server started - outputting raw Gazebo coordinates");
  RCLCPP_INFO(node->get_logger(), "Publishing raw trajectory to: /xtdrone2/planning/raw_trajectory");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}