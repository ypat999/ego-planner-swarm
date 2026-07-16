#include "bspline_opt/uniform_bspline.h"
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unistd.h>
#include <limits.h>

rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_traj_sub;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_planning_sub;

quadrotor_msgs::msg::PositionCommand pos_cmd;
geometry_msgs::msg::PoseStamped goal_pose;
bool have_goal_ = false;

// 记录当前目标点，用于判断重复目标
Eigen::Vector3d current_goal_pos_ = Eigen::Vector3d::Zero();
bool has_valid_goal_ = false;
bool goal_near_origin_ = false;

// Current drone position and orientation
Eigen::Vector3d current_pos_ = Eigen::Vector3d::Zero();
Eigen::Quaterniond current_orientation_ = Eigen::Quaterniond::Identity();
bool have_odom_ = false;

// Target yaw from goal pose orientation
double target_yaw_ = 0.0;

// TF transform variables
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::string target_frame_ = "world"; // 目标坐标系frame_id

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_ = 0.5;

// Safe zone descent configuration
bool szd_enabled_ = false;
double szd_speed_ = 0.5;
Eigen::Vector3d szd_zone_size_ = Eigen::Vector3d(2.0, 2.0, 2.0);
double szd_position_threshold_ = 0.05;

// Safe zone descent state
bool szd_active_ = false;
enum SZDPhase { SZD_NONE, SZD_HORIZONTAL, SZD_VERTICAL, SZD_DONE };
SZDPhase szd_phase_ = SZD_NONE;
Eigen::Vector3d szd_target_ = Eigen::Vector3d::Zero();
Eigen::Vector3d szd_ref_pos_ = Eigen::Vector3d::Zero();
bool szd_ref_pos_initialized_ = false;

// Safe zone descent yaw control
double szd_current_yaw_ = 0.0;
double szd_yaw_speed_ = 1.0;
constexpr double SZD_YAW_THRESHOLD = 0.02;
int szd_cmd_count_ = 0;

// Topic configuration variables
std::string odom_world_topic;
std::string grid_map_cloud_topic;
std::string grid_map_pose_topic;
std::string default_namespace;
bool default_use_sim_time;

bool isTargetInSafeZone(const Eigen::Vector3d &target)
{
  Eigen::Vector3d half_size = szd_zone_size_ / 2.0;
  return fabs(target(0)) < half_size(0) &&
         fabs(target(1)) < half_size(1) &&
         fabs(target(2)) < half_size(2);
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

void resetTrajCallback(const std_msgs::msg::Empty::ConstSharedPtr msg)
{
  (void)msg;
  if (receive_traj_)
  {
    rclcpp::Clock clock(RCL_ROS_TIME);
    start_time_ = clock.now();
    RCLCPP_INFO(rclcpp::get_logger("traj_server"), "Trajectory start time reset to now");
  }
}

void stopPlanningCallback(const std_msgs::msg::Empty::ConstSharedPtr msg)
{
  (void)msg;
  RCLCPP_WARN(rclcpp::get_logger("traj_server"), "Received /stop_planning, stopping trajectory publishing");
  receive_traj_ = false;
  szd_active_ = false;
  szd_phase_ = SZD_NONE;
  szd_ref_pos_initialized_ = false;
}

void bsplineCallback(traj_utils::msg::Bspline::ConstSharedPtr msg)
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

void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
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

void goalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
{
  // 检查目标点数据的有效性
  bool data_valid = true;
  
  // 检查位置数据
  if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y) || !std::isfinite(msg->pose.position.z))
  {
    RCLCPP_WARN(rclcpp::get_logger("traj_server"), "收到无效的目标点位置数据: x=%f, y=%f, z=%f", 
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    data_valid = false;
  }
  
  // 检查姿态数据
  double quat_norm = sqrt(msg->pose.orientation.w * msg->pose.orientation.w +
                         msg->pose.orientation.x * msg->pose.orientation.x +
                         msg->pose.orientation.y * msg->pose.orientation.y +
                         msg->pose.orientation.z * msg->pose.orientation.z);
  if (fabs(quat_norm - 1.0) > 0.01)
  {
    RCLCPP_WARN(rclcpp::get_logger("traj_server"), "收到无效的目标点姿态数据: 四元数范数=%f (期望1.0)", quat_norm);
    data_valid = false;
  }
  
  if (!data_valid)
  {
    RCLCPP_ERROR(rclcpp::get_logger("traj_server"), "目标点数据无效，跳过本次处理");
    return;
  }

  // 使用配置的目标frame
  std::string target_frame = target_frame_;

  // 检查frame_id，如果不同则进行tf变换
  geometry_msgs::msg::PoseStamped transformed_msg = *msg;
  if (msg->header.frame_id != target_frame)
  {
    try
    {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
          target_frame, 
          msg->header.frame_id,
          msg->header.stamp,
          rclcpp::Duration::from_seconds(1.0));
      
      tf2::doTransform(*msg, transformed_msg, transform);
      
      RCLCPP_DEBUG(rclcpp::get_logger("traj_server"), "目标点从frame '%s' 变换到frame '%s': 原始位置(%.2f, %.2f, %.2f) -> 变换后位置(%.2f, %.2f, %.2f)",
                  msg->header.frame_id.c_str(), target_frame.c_str(),
                  msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                  transformed_msg.pose.position.x, transformed_msg.pose.position.y, transformed_msg.pose.position.z);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(rclcpp::get_logger("traj_server"), "TF变换失败: %s (从 '%s' 到 '%s')", 
                  ex.what(), msg->header.frame_id.c_str(), target_frame.c_str());
      return;
    }
  }

  Eigen::Vector3d new_goal_pos(transformed_msg.pose.position.x, transformed_msg.pose.position.y, transformed_msg.pose.position.z);

  // 检查新目标点是否与当前目标点相同
  if (has_valid_goal_)
  {
    double position_diff = (new_goal_pos - current_goal_pos_).norm();
    if (position_diff < 0.01) // 1cm 阈值，认为是同一个点
    {
      RCLCPP_DEBUG(rclcpp::get_logger("traj_server"), 
                  "New goal pose is the same as current (diff %.4f m), ignoring request, continuing current state", 
                  position_diff);
      return;
    }
  }

  goal_pose = transformed_msg;
  have_goal_ = true;
  
  // 更新当前目标点
  current_goal_pos_ = new_goal_pos;
  has_valid_goal_ = true;
  
  // 判断目标点是否在原点附近（距离原点0.5m以内）
  goal_near_origin_ = isTargetInSafeZone(new_goal_pos);

  // Safe zone descent check
  if (szd_enabled_ && goal_near_origin_)
  {
    szd_active_ = true;
    szd_phase_ = SZD_HORIZONTAL;
    szd_target_ = new_goal_pos;
    szd_ref_pos_initialized_ = false;
    szd_cmd_count_ = 0;  // 重置命令计数器
    receive_traj_ = false;
    RCLCPP_INFO(rclcpp::get_logger("traj_server"),
                "Target (%.2f, %.2f, %.2f) is in safe zone, activating safe zone descent mode (speed=%.2f)",
                new_goal_pos(0), new_goal_pos(1), new_goal_pos(2), szd_speed_);
    RCLCPP_INFO(rclcpp::get_logger("traj_server"),
                "Safe zone descent activated: szd_active_=%s, have_odom_=%s, current_pos_=(%.2f, %.2f, %.2f)",
                szd_active_ ? "true" : "false", have_odom_ ? "true" : "false",
                current_pos_(0), current_pos_(1), current_pos_(2));
    RCLCPP_INFO(rclcpp::get_logger("traj_server"),
                "Safe zone descent: target yaw set to %.2f rad (%.2f deg)",
                target_yaw_, target_yaw_ * 180.0 / M_PI);
  }
  else
  {
    if (szd_active_)
    {
      RCLCPP_INFO(rclcpp::get_logger("traj_server"),
                  "Target (%.2f, %.2f, %.2f) is outside safe zone, deactivating safe zone descent mode",
                  new_goal_pos(0), new_goal_pos(1), new_goal_pos(2));
      szd_active_ = false;
      szd_phase_ = SZD_NONE;
    }
    // 不再设置 receive_traj_ = false，避免中断当前轨迹执行
    // 新 bspline 到达后会自然替换当前轨迹
  }

  // 从目标点的四元数中提取偏航角，存储到 target_yaw_ 供后续使用
  Eigen::Quaterniond q_goal(transformed_msg.pose.orientation.w, 
                           transformed_msg.pose.orientation.x, 
                           transformed_msg.pose.orientation.y, 
                           transformed_msg.pose.orientation.z);
  // 使用更可靠的atan2方法提取yaw，避免eulerAngles的不稳定性
  Eigen::Matrix3d rot_goal = q_goal.toRotationMatrix();
  target_yaw_ = atan2(rot_goal(1, 0), rot_goal(0, 0));
  
  RCLCPP_INFO(rclcpp::get_logger("traj_server"), 
              "Received new goal pose: (%.2f, %.2f, %.2f), target yaw from orientation: %.2f rad (%.2f deg)", 
              transformed_msg.pose.position.x, transformed_msg.pose.position.y, transformed_msg.pose.position.z,
              target_yaw_, target_yaw_ * 180.0 / M_PI);
}

// 计算当前时刻的期望偏航角及偏航角速度
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, rclcpp::Time &time_now, rclcpp::Time &time_last)
{
  constexpr double PI = 3.1415926;                 // 圆周率
  constexpr double YAW_DOT_MAX_PER_SEC = PI / 2;         // 最大偏航角速度（rad/s）
  std::pair<double, double> yaw_yawdot(0, 0);      // 返回的偏航角与偏航角速度
  double yaw = 0;                                    // 当前偏航角
  double yawdot = 0;                                 // 当前偏航角速度

  // 计算目标偏航角
  double yaw_temp;
  // 轨迹结束后使用目标偏航角
  if (t_cur >= traj_duration_ && have_goal_) {
    yaw_temp = target_yaw_;
  } else {
    // 轨迹执行中：接近目标时提前转向目标偏航角，否则跟随轨迹方向
    if (have_goal_) {
      double dist_to_goal = (Eigen::Vector2d(pos(0), pos(1))
                           - Eigen::Vector2d(current_goal_pos_(0), current_goal_pos_(1))).norm();
      if (dist_to_goal < 2.0) {
        yaw_temp = target_yaw_;  // 距目标2m内提前旋转到目标yaw
      } else {
        goto use_traj_yaw;
      }
    } else {
      use_traj_yaw:
      Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos
                : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
      yaw_temp = dir.norm() > 0.001 ? atan2(dir(1), dir(0)) : last_yaw_;
    }
  }

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
  rclcpp::Clock clock(RCL_ROS_TIME);
  rclcpp::Time time_now = clock.now();

  static int log_counter = 0;
  if (szd_active_ && log_counter % 20 == 0) // 每1秒输出一次（20*50ms=1s）
  {
    RCLCPP_DEBUG(rclcpp::get_logger("traj_server"),
                "cmdCallback: szd_active_=%s, have_odom_=%s, szd_phase_=%d, szd_ref_pos_initialized_=%s",
                szd_active_ ? "true" : "false", have_odom_ ? "true" : "false", szd_phase_,
                szd_ref_pos_initialized_ ? "true" : "false");
  }
  if (szd_active_ && !have_odom_)
  {
    RCLCPP_WARN(rclcpp::get_logger("traj_server"),
                         "Safe zone descent active but no odometry data received yet!");
  }
  log_counter++;

  if (szd_active_ && have_odom_)
  {
    double dt = 0.05;

    if (!szd_ref_pos_initialized_)
    {
      szd_ref_pos_ = current_pos_;
      szd_ref_pos_initialized_ = true;
      
      // 使用更可靠的方法提取yaw（从旋转矩阵）
      Eigen::Matrix3d rot = current_orientation_.toRotationMatrix();
      szd_current_yaw_ = atan2(rot(1, 0), rot(0, 0));
      
      RCLCPP_INFO(rclcpp::get_logger("traj_server"),
                  "Safe zone descent: starting from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
                  current_pos_(0), current_pos_(1), current_pos_(2),
                  szd_target_(0), szd_target_(1), szd_target_(2));
      RCLCPP_INFO(rclcpp::get_logger("traj_server"),
                  "Safe zone descent: initial yaw %.2f rad (%.2f deg), target yaw %.2f rad (%.2f deg), yaw_diff %.2f rad",
                  szd_current_yaw_, szd_current_yaw_ * 180.0 / M_PI,
                  target_yaw_, target_yaw_ * 180.0 / M_PI,
                  normalizeAngle(target_yaw_ - szd_current_yaw_));
    }

    Eigen::Vector3d pos_flu(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
    double yaw = last_yaw_;
    double yaw_dot = 0.0;

    if (szd_phase_ == SZD_HORIZONTAL)
    {
      Eigen::Vector3d target(szd_target_(0), szd_target_(1), szd_ref_pos_(2));
      Eigen::Vector3d dir = target - szd_ref_pos_;
      double dist = dir.norm();

      double yaw_diff = target_yaw_ - szd_current_yaw_;
      yaw_diff = normalizeAngle(yaw_diff);
      
      if (fabs(yaw_diff) < SZD_YAW_THRESHOLD)
      {
        szd_current_yaw_ = normalizeAngle(target_yaw_);
        yaw = szd_current_yaw_;
        yaw_dot = 0.0;
      }
      else
      {
        double yaw_step = szd_yaw_speed_ * dt;
        if (fabs(yaw_diff) < yaw_step)
        {
          szd_current_yaw_ = target_yaw_;
        }
        else
        {
          szd_current_yaw_ += (yaw_diff > 0 ? 1.0 : -1.0) * yaw_step;
        }
        
        szd_current_yaw_ = normalizeAngle(szd_current_yaw_);
        yaw = szd_current_yaw_;
        yaw_dot = (yaw_diff > 0 ? 1.0 : -1.0) * szd_yaw_speed_;
      }

      if (dist < szd_position_threshold_)
      {
        szd_ref_pos_ = target;
        szd_phase_ = SZD_VERTICAL;
        RCLCPP_INFO(rclcpp::get_logger("traj_server"),
                    "Safe zone descent: horizontal phase complete at (%.2f, %.2f, %.2f), starting vertical descent to z=%.2f",
                    szd_ref_pos_(0), szd_ref_pos_(1), szd_ref_pos_(2), szd_target_(2));
      }
      else
      {
        dir.normalize();
        double step = min(szd_speed_ * dt, dist);
        szd_ref_pos_ += dir * step;
        vel = dir * szd_speed_;
      }
      pos_flu = szd_ref_pos_;
    }
    else if (szd_phase_ == SZD_VERTICAL)
    {
      Eigen::Vector3d target = szd_target_;
      Eigen::Vector3d dir = target - szd_ref_pos_;
      double dist = dir.norm();

      yaw = target_yaw_;
      yaw_dot = 0.0;

      if (dist < szd_position_threshold_)
      {
        szd_ref_pos_ = target;
        szd_phase_ = SZD_DONE;
        szd_active_ = false;
        RCLCPP_INFO(rclcpp::get_logger("traj_server"),
                    "Safe zone descent: vertical descent complete, reached target (%.2f, %.2f, %.2f)",
                    szd_target_(0), szd_target_(1), szd_target_(2));
      }
      else
      {
        dir.normalize();
        double step = min(szd_speed_ * dt, dist);
        szd_ref_pos_ += dir * step;
        vel = dir * szd_speed_;
      }
      pos_flu = szd_ref_pos_;
    }
    else
    {
      pos_flu = szd_target_;
      vel.setZero();
      acc.setZero();
      yaw = target_yaw_;
      yaw_dot = 0.0;
    }

    pos_cmd.header.stamp = time_now;
    pos_cmd.header.frame_id = "world";
    pos_cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
    pos_cmd.trajectory_id = traj_id_;

    pos_cmd.position.x = pos_flu(0);
    pos_cmd.position.y = pos_flu(1);
    pos_cmd.position.z = pos_flu(2);

    pos_cmd.velocity.x = vel(0);
    pos_cmd.velocity.y = vel(1);
    pos_cmd.velocity.z = vel(2);

    pos_cmd.acceleration.x = acc(0);
    pos_cmd.acceleration.y = acc(1);
    pos_cmd.acceleration.z = acc(2);

    pos_cmd.yaw = yaw;
    pos_cmd.yaw_dot = yaw_dot;

    last_yaw_ = yaw;

    // 添加详细日志，前10次命令输出详细信息
    if (szd_cmd_count_ < 10)
    {
      RCLCPP_INFO(rclcpp::get_logger("traj_server"),
                  "SZD cmd #%d: phase=%d, pos=(%.2f, %.2f, %.2f), yaw=%.2f rad (%.2f deg), yaw_dot=%.2f rad/s",
                  szd_cmd_count_, szd_phase_,
                  pos_flu(0), pos_flu(1), pos_flu(2),
                  yaw, yaw * 180.0 / M_PI, yaw_dot);
      szd_cmd_count_++;
    }

    pos_cmd_pub->publish(pos_cmd);
    return;
  }

  /* no publishing before receive traj_ */
  if (!receive_traj_)
  {
    if (have_goal_)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("traj_server"), "Received goal pose but waiting for trajectory (bspline)...");
    }
    return;
  }

  time_now = clock.now();
  double t_cur = (time_now - start_time_).seconds();

  static double last_t_cur = 0.0;
  if (t_cur < 0.0 && receive_traj_)
  {
    RCLCPP_WARN(rclcpp::get_logger("traj_server"), "Time jump detected (t_cur=%.3f -> last_t_cur=%.3f), adjusting start_time_ to continue trajectory", t_cur, last_t_cur);
    start_time_ = time_now - rclcpp::Duration::from_seconds(last_t_cur);
    t_cur = last_t_cur;
  }
  
  if (t_cur >= 0.0 && t_cur < traj_duration_)
  {
    last_t_cur = t_cur;
  }

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
    pos_flu = traj_[0].evaluateDeBoorT(traj_duration_);
    // 正常完成：终点接近goal → 输出精确goal位置消除B样条近似误差
    // 急停打断：终点远离goal → 保持悬停避免飞向障碍物内目标
    if (have_goal_ && !goal_near_origin_)
    {
      double dist_to_goal = (pos_flu - Eigen::Vector3d(
          goal_pose.pose.position.x,
          goal_pose.pose.position.y,
          goal_pose.pose.position.z)).norm();
      if (dist_to_goal < 0.5)
      {
        pos_flu(0) = goal_pose.pose.position.x;
        pos_flu(1) = goal_pose.pose.position.y;
        pos_flu(2) = goal_pose.pose.position.z;
      }
    }
    vel.setZero();
    acc.setZero();
    pos_f = pos_flu;

    // yaw 通过 calculate_yaw 限速过渡
    yaw_yawdot = calculate_yaw(t_cur, pos_flu, time_now, time_last);
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  pos_cmd.header.stamp = time_now;
  pos_cmd.header.frame_id = "world";
  pos_cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  pos_cmd.trajectory_id = traj_id_;

  pos_cmd.position.x = pos_flu(0);
  pos_cmd.position.y = pos_flu(1);
  pos_cmd.position.z = pos_flu(2);

  pos_cmd.velocity.x = vel(0);
  pos_cmd.velocity.y = vel(1);
  pos_cmd.velocity.z = vel(2);

  pos_cmd.acceleration.x = acc(0);
  pos_cmd.acceleration.y = acc(1);
  pos_cmd.acceleration.z = acc(2);

  pos_cmd.yaw = yaw_yawdot.first;
  pos_cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = yaw_yawdot.first;

  pos_cmd_pub->publish(pos_cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");

  // Get hostname and configure topics based on host
  char hostname[HOST_NAME_MAX];
  gethostname(hostname, HOST_NAME_MAX);
  std::string hostname_str(hostname);

  if (hostname_str == "ywj-B250-D3A" || hostname_str == "DESKTOP-ypat")
  {
    default_namespace = "/x500_depth_0/";
    default_use_sim_time = true;
    odom_world_topic = "/x500_depth_0/odometry";
    grid_map_cloud_topic = "/livox_down/lidar";
    grid_map_pose_topic = "/x500_depth_0/StereoOV7251/pose";
    RCLCPP_INFO(node->get_logger(), "Running on %s, using simulation topics", hostname_str.c_str());
  }
  else
  {
    default_namespace = "/";
    default_use_sim_time = false;
    odom_world_topic = "lio/robo/odom";
    grid_map_cloud_topic = "lio/cloud_world";
    grid_map_pose_topic = "mid360/pose";
    RCLCPP_INFO(node->get_logger(), "Running on %s, using real robot topics", hostname_str.c_str());
  }

  // Get ROS namespace parameter
  std::string ros_ns;
  node->declare_parameter("ros_ns", default_namespace);
  node->get_parameter("ros_ns", ros_ns);

  //Get time_forward parameter
  node->declare_parameter("traj_server/time_forward", 0.5);
  node->get_parameter("traj_server/time_forward", time_forward_);
  
  // Get target frame parameter
  node->declare_parameter("traj_server/frame_id", "world");
  node->get_parameter("traj_server/frame_id", target_frame_);

  // Safe zone descent parameters
  node->declare_parameter("safe_zone_descent/enabled", false);
  node->get_parameter("safe_zone_descent/enabled", szd_enabled_);
  node->declare_parameter("safe_zone_descent/speed", 0.5);
  node->get_parameter("safe_zone_descent/speed", szd_speed_);
  node->declare_parameter("safe_zone_descent/yaw_speed", 1.0);
  node->get_parameter("safe_zone_descent/yaw_speed", szd_yaw_speed_);
  node->declare_parameter("safe_zone_descent/zone_size_x", 2.0);
  node->get_parameter("safe_zone_descent/zone_size_x", szd_zone_size_(0));
  node->declare_parameter("safe_zone_descent/zone_size_y", 2.0);
  node->get_parameter("safe_zone_descent/zone_size_y", szd_zone_size_(1));
  node->declare_parameter("safe_zone_descent/zone_size_z", 2.0);
  node->get_parameter("safe_zone_descent/zone_size_z", szd_zone_size_(2));
  node->declare_parameter("safe_zone_descent/position_threshold", 0.05);
  node->get_parameter("safe_zone_descent/position_threshold", szd_position_threshold_);

  // 参数运行时修改回调
  auto param_handle = node->add_on_set_parameters_callback(
    [](const std::vector<rclcpp::Parameter> & params) {
      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (name == "traj_server/time_forward") time_forward_ = p.as_double();
        else if (name == "traj_server/frame_id") target_frame_ = p.as_string();
        else if (name == "safe_zone_descent/enabled") szd_enabled_ = p.as_bool();
        else if (name == "safe_zone_descent/speed") szd_speed_ = p.as_double();
        else if (name == "safe_zone_descent/yaw_speed") szd_yaw_speed_ = p.as_double();
        else if (name == "safe_zone_descent/zone_size_x") szd_zone_size_(0) = p.as_double();
        else if (name == "safe_zone_descent/zone_size_y") szd_zone_size_(1) = p.as_double();
        else if (name == "safe_zone_descent/zone_size_z") szd_zone_size_(2) = p.as_double();
        else if (name == "safe_zone_descent/position_threshold") szd_position_threshold_ = p.as_double();
      }
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
    });

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
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
      odom_world_topic,
      10,
      odomCallback);

  // Subscribe to goal pose
  goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose_3d",
      10,
      goalCallback);

  pos_cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      std::string("/xtdrone2") + ros_ns + "cmd_trajectory_flu",
      50);

  reset_traj_sub = node->create_subscription<std_msgs::msg::Empty>(
      std::string("/xtdrone2") + ros_ns + "reset_traj_time",
      10,
      resetTrajCallback);

  stop_planning_sub = node->create_subscription<std_msgs::msg::Empty>(
      "/stop_planning",
      1,
      stopPlanningCallback);

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(50),
      cmdCallback);

  RCLCPP_INFO(node->get_logger(), "Trajectory server started - outputting position, velocity, acceleration");
  RCLCPP_INFO(node->get_logger(), "Publishing trajectory to: /xtdrone2%s cmd_trajectory_flu", ros_ns.c_str());
  RCLCPP_INFO(node->get_logger(), "Subscribed to odometry: %s", odom_world_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "Subscribed to goal pose: /goal_pose_3d");
  RCLCPP_INFO(node->get_logger(), "Subscribed to grid map cloud: %s", grid_map_cloud_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "Subscribed to grid map pose: %s", grid_map_pose_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "Safe zone descent: enabled=%s, speed=%.2f m/s, yaw_speed=%.2f rad/s, zone_size=[%.2f, %.2f, %.2f], threshold=%.3f",
              szd_enabled_ ? "true" : "false", szd_speed_, szd_yaw_speed_,
              szd_zone_size_(0), szd_zone_size_(1), szd_zone_size_(2), szd_position_threshold_);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}