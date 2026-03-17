#include "bspline_opt/uniform_bspline.h"
#include "traj_utils/msg/bspline.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_traj_pub;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

geometry_msgs::msg::PoseStamped raw_cmd;
geometry_msgs::msg::PoseStamped goal_pose;
bool have_goal_ = false;

// Current drone position and orientation
Eigen::Vector3d current_pos_ = Eigen::Vector3d::Zero();
Eigen::Quaterniond current_orientation_ = Eigen::Quaterniond::Identity();
bool have_odom_ = false;

// Turning state variables
bool turning_to_goal_ = false;
double target_yaw_ = 0.0;

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_ = 0.5;

void bsplineCallback(traj_utils::msg::Bspline::ConstPtr msg)
{
  // 如果正在转向目标点，收到轨迹后停止转向
  if (turning_to_goal_)
  {
    turning_to_goal_ = false;
    RCLCPP_INFO(rclcpp::get_logger("traj_server"), 
                "Received trajectory, stopping turn and continuing with path planning");
  }

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

void odomCallback(const nav_msgs::msg::Odometry::ConstPtr &msg)
{
  current_pos_(0) = msg->pose.pose.position.x;
  current_pos_(1) = msg->pose.pose.position.y;
  current_pos_(2) = msg->pose.pose.position.z;
  
  current_orientation_ = Eigen::Quaterniond(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z
  );
  
  have_odom_ = true;
}

void goalCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &msg)
{
  goal_pose = *msg;
  have_goal_ = true;
  
  // 每次收到目标点后，重置轨迹接收标志，确保检查新的轨迹
  receive_traj_ = false;
  
  // 如果有当前位置信息，立即计算目标角度并开始转向
  if (have_odom_)
  {
    Eigen::Vector3d goal_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Vector3d direction = goal_pos - current_pos_;
    
    // 计算目标偏航角（忽略高度差，只考虑水平方向）
    if (direction.norm() > 0.001)
    {
      target_yaw_ = atan2(direction(1), direction(0));
      turning_to_goal_ = true;
      
      RCLCPP_INFO(rclcpp::get_logger("traj_server"), 
                  "Received new goal pose: (%.2f, %.2f, %.2f), calculating target yaw: %.2f rad (%.2f deg)", 
                  msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                  target_yaw_, target_yaw_ * 180.0 / M_PI);
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("traj_server"), 
                  "Goal too close to current position, skipping turn");
    }
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("traj_server"), 
                "Received new goal pose: (%.2f, %.2f, %.2f), waiting for odometry data", 
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }
}

// 计算当前时刻的期望偏航角及偏航角速度
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;                 // 圆周率
  constexpr double YAW_DOT_MAX_PER_SEC = PI / 4;         // 最大偏航角速度（rad/s）
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
  // 统一时间源
  rclcpp::Clock clock(RCL_ROS_TIME);  
  rclcpp::Time time_now = clock.now();
  
  // 如果正在转向目标点，优先处理转向逻辑
  if (turning_to_goal_ && have_odom_)
  {
    // 计算当前偏航角
    Eigen::Vector3d euler = current_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
    double current_yaw = euler(2);
    
    // 计算偏航角误差
    double yaw_error = target_yaw_ - current_yaw;
    
    // 处理角度跨越 ±PI 的情况
    if (yaw_error > M_PI)
      yaw_error -= 2 * M_PI;
    else if (yaw_error < -M_PI)
      yaw_error += 2 * M_PI;
    
    // 检查是否已经朝向目标点（误差小于阈值）
    constexpr double YAW_TOLERANCE = 0.087; // 约5度
    if (fabs(yaw_error) < YAW_TOLERANCE)
    {
      turning_to_goal_ = false;
      RCLCPP_INFO(rclcpp::get_logger("traj_server"), 
                  "Turn completed! Current yaw: %.2f°, Target yaw: %.2f°", 
                  current_yaw * 180.0 / M_PI, target_yaw_ * 180.0 / M_PI);
    }
    else
    {
      // 发布转向命令：保持当前位置，只改变偏航角
      raw_cmd.header.stamp = time_now;
      raw_cmd.header.frame_id = "world";
      raw_cmd.pose.position.x = current_pos_(0);
      raw_cmd.pose.position.y = current_pos_(1);
      raw_cmd.pose.position.z = current_pos_(2);
      
      // 使用目标偏航角
      Eigen::Quaterniond q_flu = Eigen::Quaterniond(Eigen::AngleAxisd(target_yaw_, Eigen::Vector3d::UnitZ()));
      raw_cmd.pose.orientation.x = q_flu.x();
      raw_cmd.pose.orientation.y = q_flu.y();
      raw_cmd.pose.orientation.z = q_flu.z();
      raw_cmd.pose.orientation.w = q_flu.w();
      
      raw_traj_pub->publish(raw_cmd);
      
      // 更新last_yaw_以保持一致性
      last_yaw_ = target_yaw_;
      
      return;
    }
  }
  
  /* no publishing before receive traj_ */
  if (!receive_traj_)
  {
    if (have_goal_)
    {
      RCLCPP_INFO(rclcpp::get_logger("traj_server"), "Received goal pose but waiting for trajectory (bspline)...");
    }
    return;
  }

  // 统一时间源
  rclcpp::Clock clock(RCL_ROS_TIME);  
  rclcpp::Time time_now = clock.now();
  double t_cur = (time_now - start_time_).seconds();

  Eigen::Vector3d pos_flu(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static rclcpp::Time time_last = clock.now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos_flu = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos_flu, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + time_forward_);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
  {
    /* use goal pose when finish traj_ */
    if (have_goal_)
    {
      // 使用目标点的位置和姿态
      pos_flu(0) = goal_pose.pose.position.x;
      pos_flu(1) = goal_pose.pose.position.y;
      pos_flu(2) = goal_pose.pose.position.z;
      
      // 从目标点的四元数中提取偏航角
      Eigen::Quaterniond q_goal(goal_pose.pose.orientation.w, 
                               goal_pose.pose.orientation.x, 
                               goal_pose.pose.orientation.y, 
                               goal_pose.pose.orientation.z);
      Eigen::Vector3d euler = q_goal.toRotationMatrix().eulerAngles(0, 1, 2);
      yaw_yawdot.first = euler(2);
      yaw_yawdot.second = 0;
      
      // RCLCPP_INFO(rclcpp::get_logger("traj_server"), "Using goal pose: (%.2f, %.2f, %.2f), yaw: %.2f", 
      //             pos_flu(0), pos_flu(1), pos_flu(2), yaw_yawdot.first);
    }
    else
    {
      // 如果没有目标点，使用轨迹终点
      pos_flu = traj_[0].evaluateDeBoorT(traj_duration_);
      yaw_yawdot.first = last_yaw_;
      yaw_yawdot.second = 0;
      
      RCLCPP_INFO(rclcpp::get_logger("traj_server"), "Using trajectory end point: (%.2f, %.2f, %.2f)", 
                  pos_flu(0), pos_flu(1), pos_flu(2));
    }
    
    vel.setZero();
    acc.setZero();
    pos_f = pos_flu;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  // 将偏航角转换为四元数
  // 在flu坐标系下构造yaw四元数（绕Z轴）
  Eigen::Quaterniond q_flu = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_yawdot.first, Eigen::Vector3d::UnitZ()));

  // 设置原始Gazebo坐标系下的位置和姿态
  raw_cmd.header.stamp = time_now;
  raw_cmd.header.frame_id = "world";
  raw_cmd.pose.position.x = pos_flu(0);  // flu X
  raw_cmd.pose.position.y = pos_flu(1);  // flu Y
  raw_cmd.pose.position.z = pos_flu(2);  // flu Z
  
  raw_cmd.pose.orientation.x = q_flu.x();
  raw_cmd.pose.orientation.y = q_flu.y();
  raw_cmd.pose.orientation.z = q_flu.z();
  raw_cmd.pose.orientation.w = q_flu.w();

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
  
  if (ros_ns.empty()) {
    RCLCPP_WARN(node->get_logger(), "ROS namespace not specified, using default topics");
    ros_ns = "/";
  }

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  // Subscribe to odometry for current position
  odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom_world",
      10,
      odomCallback);

  // Subscribe to goal pose
  goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose_3d",
      10,
      goalCallback);

  // Publish raw trajectory in Gazebo coordinates
  raw_traj_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      // "/xtdrone2/planning/raw_trajectory",
      std::string("/xtdrone2") + ros_ns + "cmd_pose_local_flu",
      50);

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(50),
      cmdCallback);

  RCLCPP_INFO(node->get_logger(), "Trajectory server started - outputting raw Gazebo coordinates");
  RCLCPP_INFO(node->get_logger(), "Publishing raw trajectory to: /xtdrone2/planning/raw_trajectory");
  RCLCPP_INFO(node->get_logger(), "Subscribed to odometry: odom_world");
  RCLCPP_INFO(node->get_logger(), "Subscribed to goal pose: /goal_pose_3d");

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}