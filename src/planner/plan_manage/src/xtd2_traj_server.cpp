#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "traj_utils/msg/bspline.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <deque>
#include <mutex>

rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pos_cmd_pub;

geometry_msgs::msg::Pose cmd;

using ego_planner::UniformBspline;

bool receive_traj_ = false;
vector<UniformBspline> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_, last_yaw_dot_;
double time_forward_ = 0.5;

// Odometry monitoring and compensation
std::mutex odom_mutex_;
struct OdometryData {
    rclcpp::Time timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angular_velocity;
};

std::deque<OdometryData> gazebo_odom_buffer_;
std::deque<OdometryData> px4_odom_buffer_;

// Coordinate system alignment parameters
Eigen::Matrix3d gazebo_to_ros2_rotation_ = Eigen::Matrix3d::Identity();
Eigen::Matrix3d px4_to_ros2_rotation_ = Eigen::Matrix3d::Identity();
Eigen::Vector3d gazebo_to_ros2_translation_ = Eigen::Vector3d::Zero();
Eigen::Vector3d px4_to_ros2_translation_ = Eigen::Vector3d::Zero();

// Function declarations
void gazeboOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
void px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
void calculateOdometryCompensation();
void updateCoordinateTransformations();
Eigen::Vector3d applyPositionCompensation(const Eigen::Vector3d& original_position);
Eigen::Quaterniond applyOrientationCompensation(const Eigen::Quaterniond& original_orientation);

// Compensation parameters
bool enable_compensation_ = true;
double compensation_alpha_ = 0.8;  // Low-pass filter coefficient
Eigen::Vector3d position_compensation_ = Eigen::Vector3d::Zero();
Eigen::Quaterniond orientation_compensation_ = Eigen::Quaterniond::Identity();

// Subscribers
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gazebo_odom_sub_;
rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;

// TF buffer and listener
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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

// Gazebo odometry callback
void gazeboOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    OdometryData odom_data;
    odom_data.timestamp = msg->header.stamp;
    odom_data.position = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
    odom_data.orientation = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    odom_data.velocity = Eigen::Vector3d(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z
    );
    odom_data.angular_velocity = Eigen::Vector3d(
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z
    );
    
    // Add to buffer (keep last 10 messages)
    gazebo_odom_buffer_.push_back(odom_data);
    if (gazebo_odom_buffer_.size() > 10) {
        gazebo_odom_buffer_.pop_front();
    }
    
    // Print Gazebo position
    static rclcpp::Node::SharedPtr log_node = rclcpp::Node::make_shared("traj_server_log");
    // RCLCPP_INFO(log_node->get_logger(), "Gazebo odometry received: position=(%.3f, %.3f, %.3f)",
    //             odom_data.position(0), odom_data.position(1), odom_data.position(2));
    
    // Calculate compensation if we have PX4 data
    if (!px4_odom_buffer_.empty()) {
        calculateOdometryCompensation();
    }
}

// PX4 odometry callback
void px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  // Print PX4 position
    // static rclcpp::Node::SharedPtr log_node = rclcpp::Node::make_shared("traj_server_log");
    // RCLCPP_INFO(log_node->get_logger(), "PX4 odometry received: position=(%.3f, %.3f, %.3f)",
    //             msg->position[0], msg->position[1], msg->position[2]);


    std::lock_guard<std::mutex> lock(odom_mutex_);
    
    OdometryData odom_data;
    // Convert timestamp from PX4's uint64_t (microseconds) to rclcpp::Time
    rclcpp::Time timestamp = rclcpp::Time(msg->timestamp / 1000000, (msg->timestamp % 1000000) * 1000);
    odom_data.timestamp = timestamp;
    
    // PX4 VehicleOdometry uses NED coordinate system
    odom_data.position = Eigen::Vector3d(
        msg->position[0],  // North
        msg->position[1],  // East
        msg->position[2]   // Down
    );
    
    // PX4 VehicleOdometry uses quaternion in w, x, y, z order
    odom_data.orientation = Eigen::Quaterniond(
        msg->q[0],  // w
        msg->q[1],  // x
        msg->q[2],  // y
        msg->q[3]   // z
    );
    
    // Velocity in NED
    odom_data.velocity = Eigen::Vector3d(
        msg->velocity[0],  // North
        msg->velocity[1],  // East
        msg->velocity[2]   // Down
    );
    
    // Angular velocity in NED
    odom_data.angular_velocity = Eigen::Vector3d(
        msg->angular_velocity[0],  // Roll rate
        msg->angular_velocity[1],  // Pitch rate
        msg->angular_velocity[2]   // Yaw rate
    );
    
    // Add to buffer (keep last 10 messages)
    px4_odom_buffer_.push_back(odom_data);
    if (px4_odom_buffer_.size() > 10) {
        px4_odom_buffer_.pop_front();
    }
    
    // Calculate compensation if we have Gazebo data
    if (!gazebo_odom_buffer_.empty()) {
        calculateOdometryCompensation();
    }
}

// Calculate odometry compensation between Gazebo and PX4
void calculateOdometryCompensation()
{
    if (gazebo_odom_buffer_.empty() || px4_odom_buffer_.empty()) {
        return;
    }
    
    // Update coordinate transformations using TF
    updateCoordinateTransformations();
    
    // Get the latest odometry data
    OdometryData gazebo_odom = gazebo_odom_buffer_.back();
    OdometryData px4_odom = px4_odom_buffer_.back();
    
    // Align coordinate systems
    Eigen::Vector3d gazebo_pos_ros2 = gazebo_to_ros2_rotation_ * gazebo_odom.position + gazebo_to_ros2_translation_;
    Eigen::Vector3d px4_pos_ros2 = px4_to_ros2_rotation_ * px4_odom.position + px4_to_ros2_translation_;
    
    // Calculate px4 position in gazebo coordinate system
    Eigen::Vector3d px4_pos_gazebo = gazebo_to_ros2_rotation_.inverse() * (px4_pos_ros2 - gazebo_to_ros2_translation_);
    
    // Print compensation info
    // static rclcpp::Node::SharedPtr log_node = rclcpp::Node::make_shared("traj_server_log");
    // RCLCPP_INFO(log_node->get_logger(), "Compensation - Gazebo ROS2: (%.3f, %.3f, %.3f), PX4 ROS2: (%.3f, %.3f, %.3f)",
    //             gazebo_pos_ros2(0), gazebo_pos_ros2(1), gazebo_pos_ros2(2),
    //             px4_pos_ros2(0), px4_pos_ros2(1), px4_pos_ros2(2));
    
    // RCLCPP_INFO(log_node->get_logger(), "Compensation - PX4 in Gazebo frame: (%.3f, %.3f, %.3f)",
    //             px4_pos_gazebo(0), px4_pos_gazebo(1), px4_pos_gazebo(2));
    
    Eigen::Quaterniond gazebo_ori_ros2 = Eigen::Quaterniond(gazebo_to_ros2_rotation_) * gazebo_odom.orientation;
    Eigen::Quaterniond px4_ori_ros2 = Eigen::Quaterniond(px4_to_ros2_rotation_) * px4_odom.orientation;
    
    // Calculate position difference
    Eigen::Vector3d position_diff = px4_pos_ros2 - gazebo_pos_ros2;
    
    // RCLCPP_INFO(log_node->get_logger(), "Compensation - Position difference: (%.3f, %.3f, %.3f)",
    //             position_diff(0), position_diff(1), position_diff(2));
    
    // Calculate orientation difference
    Eigen::Quaterniond orientation_diff = px4_ori_ros2 * gazebo_ori_ros2.inverse();
    
    // Apply low-pass filter to smooth compensation
    position_compensation_ = compensation_alpha_ * position_compensation_ + (1.0 - compensation_alpha_) * position_diff;
    
    // For orientation, use spherical linear interpolation
    double dot_product = orientation_compensation_.dot(orientation_diff);
    if (dot_product < 0.0) {
        orientation_diff = Eigen::Quaterniond(-orientation_diff.w(), -orientation_diff.x(), 
                                             -orientation_diff.y(), -orientation_diff.z());
    }
    
    orientation_compensation_ = orientation_compensation_.slerp(1.0 - compensation_alpha_, orientation_diff);
    
    // Print current compensation values
    // RCLCPP_INFO(log_node->get_logger(), "Compensation - Current values: position=(%.3f, %.3f, %.3f), orientation=(x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
    //             position_compensation_(0), position_compensation_(1), position_compensation_(2),
    //             orientation_compensation_.x(), orientation_compensation_.y(), 
    //             orientation_compensation_.z(), orientation_compensation_.w());
}

// Apply compensation to trajectory position
Eigen::Vector3d applyPositionCompensation(const Eigen::Vector3d& original_position)
{
    if (!enable_compensation_) {
        return original_position;
    }
    
    return original_position + position_compensation_;
}

// Apply compensation to trajectory orientation
Eigen::Quaterniond applyOrientationCompensation(const Eigen::Quaterniond& original_orientation)
{
    if (!enable_compensation_) {
        return original_orientation;
    }
    
    return orientation_compensation_ * original_orientation;
}

// Update coordinate transformations using TF
void updateCoordinateTransformations()
{
    try {
        // Try to get transformation from Gazebo frame to ROS2 frame
        geometry_msgs::msg::TransformStamped gazebo_tf;
        gazebo_tf = tf_buffer_->lookupTransform("world", "gazebo_world", tf2::TimePointZero);
        
        Eigen::Quaterniond gazebo_rot(
            gazebo_tf.transform.rotation.w,
            gazebo_tf.transform.rotation.x,
            gazebo_tf.transform.rotation.y,
            gazebo_tf.transform.rotation.z
        );
        gazebo_to_ros2_rotation_ = gazebo_rot.toRotationMatrix();
        gazebo_to_ros2_translation_ = Eigen::Vector3d(
            gazebo_tf.transform.translation.x,
            gazebo_tf.transform.translation.y,
            gazebo_tf.transform.translation.z
        );
    } catch (tf2::TransformException &ex) {
        // If TF not available, use default transformations
    }
    
    try {
        // Try to get transformation from PX4 frame to ROS2 frame
        geometry_msgs::msg::TransformStamped px4_tf;
        px4_tf = tf_buffer_->lookupTransform("world", "px4_world", tf2::TimePointZero);
        
        Eigen::Quaterniond px4_rot(
            px4_tf.transform.rotation.w,
            px4_tf.transform.rotation.x,
            px4_tf.transform.rotation.y,
            px4_tf.transform.rotation.z
        );
        px4_to_ros2_rotation_ = px4_rot.toRotationMatrix();
        px4_to_ros2_translation_ = Eigen::Vector3d(
            px4_tf.transform.translation.x,
            px4_tf.transform.translation.y,
            px4_tf.transform.translation.z
        );
    } catch (tf2::TransformException &ex) {
        // If TF not available, use default transformations
    }
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
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos_enu = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** calculate yaw ***/
    yaw_yawdot = calculate_yaw(t_cur, pos_enu, time_now, time_last);
    /*** calculate yaw ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);
  }
  else if (t_cur >= traj_duration_)
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

  // Apply odometry compensation
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    pos_enu = applyPositionCompensation(pos_enu);
    // Note: Yaw calculation is already done, so we don't apply orientation compensation to yaw
    // The orientation compensation is applied to the final quaternion conversion below
  }

  // 将偏航角转换为四元数
  Eigen::Quaterniond q_ned;
  Eigen::AngleAxisd yaw_angle_ned(yaw_yawdot.first + M_PI/2, Eigen::Vector3d::UnitX());
  q_ned = Eigen::Quaterniond(yaw_angle_ned);

  // Apply orientation compensation
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    q_ned = applyOrientationCompensation(q_ned);
  }

  cmd.position.x = pos_enu(1);      // FLU Y左 -> NED X北
  cmd.position.y = pos_enu(0);      // FLU X前 -> NED Y东
  cmd.position.z = -pos_enu(2);     // FLU Z上 -> NED Z地（取反）

  
  cmd.orientation.x = q_ned.x();
  cmd.orientation.y = q_ned.y();
  cmd.orientation.z = q_ned.z();
  cmd.orientation.w = q_ned.w();

  last_yaw_ = yaw_yawdot.first;

  pos_cmd_pub->publish(cmd);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_server");

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Get ROS namespace parameter
  std::string ros_ns;
  node->declare_parameter("ros_ns", "");
  node->get_parameter("ros_ns", ros_ns);
  
  if (ros_ns.empty()) {
    RCLCPP_WARN(node->get_logger(), "ROS namespace not specified, using default topics");
    ros_ns = "";
  }

  // Create odometry subscribers
  std::string gazebo_odom_topic = "/x500_depth_0/odometry";
  std::string px4_odom_topic = "/x500_depth_0/fmu/out/vehicle_odometry";
  
  gazebo_odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      gazebo_odom_topic,
      10,
      gazeboOdomCallback);

  px4_odom_sub_ = node->create_subscription<px4_msgs::msg::VehicleOdometry>(
      px4_odom_topic,
      rclcpp::QoS(1).best_effort().transient_local(),
      px4OdomCallback);

  auto bspline_sub = node->create_subscription<traj_utils::msg::Bspline>(
      "planning/bspline",
      10,
      bsplineCallback);

  pos_cmd_pub = node->create_publisher<geometry_msgs::msg::Pose>(
      "/xtdrone2/planning/cmd_pose_local_ned",
      50);

  // Initialize coordinate system transformations
  // Gazebo to ROS2: typically identity (Gazebo uses ENU, same as ROS2)
  gazebo_to_ros2_rotation_ = Eigen::Matrix3d::Identity();
  gazebo_to_ros2_translation_ = Eigen::Vector3d::Zero();
  
  // PX4 to ROS2: PX4 uses NED, ROS2 uses ENU
  // NED to ENU: x_north -> y_east, y_east -> x_north, z_down -> -z_up
  px4_to_ros2_rotation_ << 0, 1, 0,
                          1, 0, 0,
                          0, 0, -1;
  px4_to_ros2_translation_ = Eigen::Vector3d::Zero();

  auto cmd_timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      cmdCallback);

  // Add timer to calculate compensation every second and print it
  auto compensation_timer = node->create_wall_timer(
      std::chrono::seconds(1),
      [node]() {
          calculateOdometryCompensation();
          std::lock_guard<std::mutex> lock(odom_mutex_);
          RCLCPP_INFO(node->get_logger(), "Odometry compensation: position=(%.3f, %.3f, %.3f), orientation=(x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
                      position_compensation_(0), position_compensation_(1), position_compensation_(2),
                      orientation_compensation_.x(), orientation_compensation_.y(), 
                      orientation_compensation_.z(), orientation_compensation_.w());
      });

  RCLCPP_INFO(node->get_logger(), "Trajectory server started with odometry compensation");
  RCLCPP_INFO(node->get_logger(), "Gazebo odometry topic: %s", gazebo_odom_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "PX4 odometry topic: %s", px4_odom_topic.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}