#include "plan_env/grid_map.h"
#include <limits>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  // 初始化TF2缓冲区（全局缓存，避免重复创建）
  if (!tf_buffer_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(node_->get_logger(), "TF2缓冲区初始化完成");
  }

  /* get parameter */
  double x_size, y_size, z_size;
  node_->declare_parameter("grid_map/resolution", -1.0);
  node_->declare_parameter("grid_map/map_size_x", -1.0);
  node_->declare_parameter("grid_map/map_size_y", -1.0);
  node_->declare_parameter("grid_map/map_size_z", -1.0);
  node_->declare_parameter("grid_map/local_update_range_x", -1.0);
  node_->declare_parameter("grid_map/local_update_range_y", -1.0);
  node_->declare_parameter("grid_map/local_update_range_z", -1.0);
  node_->declare_parameter("grid_map/clear_range_x", -1.0);
  node_->declare_parameter("grid_map/clear_range_y", -1.0);
  node_->declare_parameter("grid_map/clear_range_z", -1.0);
  node_->declare_parameter("grid_map/obstacles_inflation", -1.0);
  node_->declare_parameter("grid_map/obstacles_inflation_z", 0.2);
  node_->declare_parameter("grid_map/fx", -1.0);
  node_->declare_parameter("grid_map/fy", -1.0);
  node_->declare_parameter("grid_map/cx", -1.0);
  node_->declare_parameter("grid_map/cy", -1.0);
  node_->declare_parameter("grid_map/use_depth_filter", true);
  node_->declare_parameter("grid_map/depth_filter_tolerance", -1.0);
  node_->declare_parameter("grid_map/depth_filter_maxdist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_mindist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_margin", -1);
  node_->declare_parameter("grid_map/k_depth_scaling_factor", -1.0);
  node_->declare_parameter("grid_map/skip_pixel", -1);
  node_->declare_parameter("grid_map/p_hit", 0.70);
  node_->declare_parameter("grid_map/p_miss", 0.35);
  node_->declare_parameter("grid_map/p_min", 0.12);
  node_->declare_parameter("grid_map/p_max", 0.97);
  node_->declare_parameter("grid_map/p_occ", 0.80);
  node_->declare_parameter("grid_map/min_ray_length", -0.1);
  node_->declare_parameter("grid_map/max_ray_length", -0.1);
  node_->declare_parameter("grid_map/visualization_truncate_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yp", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yn", -0.1);
  node_->declare_parameter("grid_map/show_occ_time", false);
  node_->declare_parameter("grid_map/pose_type", 1);
  node_->declare_parameter("grid_map/frame_id", "world");
  node_->declare_parameter("grid_map/local_map_margin", 1);
  node_->declare_parameter("grid_map/ground_height", 1.0);
  node_->declare_parameter("grid_map/odom_depth_timeout", 1.0);
  node_->declare_parameter("grid_map/reset_buffer_enabled", true);
  node_->declare_parameter("grid_map/enable_depth_raycast", true);
  node_->declare_parameter("grid_map/origin_safe_zone_enabled", true);
  node_->declare_parameter("grid_map/origin_safe_zone_size_x", 1.0);
  node_->declare_parameter("grid_map/origin_safe_zone_size_y", 1.0);
  node_->declare_parameter("grid_map/origin_safe_zone_size_z", 1.0);

  node_->get_parameter("grid_map/resolution", mp_.resolution_);
  node_->get_parameter("grid_map/map_size_x", x_size);
  node_->get_parameter("grid_map/map_size_y", y_size);
  node_->get_parameter("grid_map/map_size_z", z_size);
  node_->get_parameter("grid_map/local_update_range_x", mp_.local_update_range_(0));
  node_->get_parameter("grid_map/local_update_range_y", mp_.local_update_range_(1));
  node_->get_parameter("grid_map/local_update_range_z", mp_.local_update_range_(2));
  // 点云模式：加(mark)复用 local_update_range，减(clear)用小范围以兼顾近端动态。
  // clear 未配置(<=0)时退化为 local_update_range，保持原行为；clear 不得超过 local_update_range，
  // 否则会擦掉未被重新标记的记忆，形成安全空洞
  double clear_x, clear_y, clear_z;
  node_->get_parameter("grid_map/clear_range_x", clear_x);
  node_->get_parameter("grid_map/clear_range_y", clear_y);
  node_->get_parameter("grid_map/clear_range_z", clear_z);
  mp_.clear_range_(0) = clear_x > 0.0 ? clear_x : mp_.local_update_range_(0);
  mp_.clear_range_(1) = clear_y > 0.0 ? clear_y : mp_.local_update_range_(1);
  mp_.clear_range_(2) = clear_z > 0.0 ? clear_z : mp_.local_update_range_(2);
  if (mp_.clear_range_(0) > mp_.local_update_range_(0))
    mp_.clear_range_(0) = mp_.local_update_range_(0);
  if (mp_.clear_range_(1) > mp_.local_update_range_(1))
    mp_.clear_range_(1) = mp_.local_update_range_(1);
  if (mp_.clear_range_(2) > mp_.local_update_range_(2))
    mp_.clear_range_(2) = mp_.local_update_range_(2);
  RCLCPP_INFO(node_->get_logger(),
              "点云更新范围: mark(local_update)=[%.1f, %.1f, %.1f], clear=[%.1f, %.1f, %.1f]",
              mp_.local_update_range_(0), mp_.local_update_range_(1), mp_.local_update_range_(2),
              mp_.clear_range_(0), mp_.clear_range_(1), mp_.clear_range_(2));
  node_->get_parameter("grid_map/obstacles_inflation", mp_.obstacles_inflation_);
  node_->get_parameter("grid_map/obstacles_inflation_z", mp_.obstacles_inflation_z_);
  node_->get_parameter("grid_map/fx", mp_.fx_);
  node_->get_parameter("grid_map/fy", mp_.fy_);
  node_->get_parameter("grid_map/cx", mp_.cx_);
  node_->get_parameter("grid_map/cy", mp_.cy_);
  node_->get_parameter("grid_map/use_depth_filter", mp_.use_depth_filter_);
  node_->get_parameter("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_);
  node_->get_parameter("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_);
  node_->get_parameter("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_);
  node_->get_parameter("grid_map/depth_filter_margin", mp_.depth_filter_margin_);
  node_->get_parameter("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_);
  node_->get_parameter("grid_map/skip_pixel", mp_.skip_pixel_);
  node_->get_parameter("grid_map/p_hit", mp_.p_hit_);
  node_->get_parameter("grid_map/p_miss", mp_.p_miss_);
  node_->get_parameter("grid_map/p_min", mp_.p_min_);
  node_->get_parameter("grid_map/p_max", mp_.p_max_);
  node_->get_parameter("grid_map/p_occ", mp_.p_occ_);
  node_->get_parameter("grid_map/min_ray_length", mp_.min_ray_length_);
  node_->get_parameter("grid_map/max_ray_length", mp_.max_ray_length_);
  node_->get_parameter("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_);
  node_->get_parameter("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_);
  node_->get_parameter("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_);
  node_->get_parameter("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_);
  node_->get_parameter("grid_map/show_occ_time", mp_.show_occ_time_);
  node_->get_parameter("grid_map/pose_type", mp_.pose_type_);
  node_->get_parameter("grid_map/frame_id", mp_.frame_id_);
  node_->get_parameter("grid_map/local_map_margin", mp_.local_map_margin_);
  node_->get_parameter("grid_map/ground_height", mp_.ground_height_);
  node_->get_parameter("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_);
  node_->get_parameter("grid_map/reset_buffer_enabled", mp_.reset_buffer_enabled_);
  node_->get_parameter("grid_map/enable_depth_raycast", mp_.enable_depth_raycast_);
  node_->get_parameter("grid_map/origin_safe_zone_enabled", mp_.origin_safe_zone_enabled_);
  node_->get_parameter("grid_map/origin_safe_zone_size_x", mp_.origin_safe_zone_size_(0));
  node_->get_parameter("grid_map/origin_safe_zone_size_y", mp_.origin_safe_zone_size_(1));
  node_->get_parameter("grid_map/origin_safe_zone_size_z", mp_.origin_safe_zone_size_(2));

  /* cloud 模式多层级地图：coarse 分辨率必须为 fine(grid_map/resolution) 的整数倍 */
  node_->declare_parameter("grid_map/coarse_resolution", 0.2);
  double coarse_res = 0.2;
  node_->get_parameter("grid_map/coarse_resolution", coarse_res);
  mp_.fine_resolution_ = mp_.resolution_;
  mp_.fine_resolution_inv_ = mp_.resolution_inv_;
  mp_.multilevel_ = !mp_.enable_depth_raycast_;
  if (mp_.multilevel_)
  {
    int ratio = (int)lround(coarse_res / mp_.fine_resolution_);
    if (ratio < 2 || fabs(coarse_res - ratio * mp_.fine_resolution_) > 1e-4 * mp_.fine_resolution_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "[grid_map] coarse_resolution %.3f 不是 fine %.3f 的整数倍, 强制 ratio=4 (%.3f)",
                  coarse_res, mp_.fine_resolution_, 4 * mp_.fine_resolution_);
      ratio = 4;
      coarse_res = ratio * mp_.fine_resolution_;
    }
    mp_.coarse_ratio_ = ratio;
    mp_.coarse_resolution_ = coarse_res;
  }
  else
  {
    mp_.coarse_ratio_ = 1;
    mp_.coarse_resolution_ = mp_.resolution_;
  }
  mp_.coarse_resolution_inv_ = 1.0 / mp_.coarse_resolution_;

  if (mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  // 膨胀按整格量化，配置值与真实生效值常不一致，启动时打印避免“配了却没生效”的反复排查
  int inf_step_log = max(0, (int)ceil(mp_.obstacles_inflation_ / mp_.resolution_));
  int inf_step_z_log = max(0, (int)ceil(mp_.obstacles_inflation_z_ / mp_.resolution_));
  RCLCPP_INFO(node_->get_logger(),
              "膨胀量化: 横向%.2fm->%d格(%.2fm/侧), 纵向%.2fm->%d格(%.2fm/侧), res=%.3fm, 体素%d x %d x %d",
              mp_.obstacles_inflation_, inf_step_log, inf_step_log * mp_.resolution_,
              mp_.obstacles_inflation_z_, inf_step_z_log, inf_step_z_log * mp_.resolution_,
              mp_.resolution_, mp_.map_voxel_num_(0), mp_.map_voxel_num_(1), mp_.map_voxel_num_(2));

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers

  // 必须用 size_t：400m×400m×20m @0.1m 的体素数约 3.2e9，int 相乘会溢出成负数
  size_t buffer_size = (size_t)mp_.map_voxel_num_(0) * (size_t)mp_.map_voxel_num_(1) *
                       (size_t)mp_.map_voxel_num_(2);

  if (mp_.multilevel_)
  {
    // 多层级：全图 inflate 缓冲不分配（fine 分辨率下单层全图会爆炸），
    // 改为 coarse 全局层(整数倍粗分辨率, 持久记忆) + fine 环形滚动窗口层
    md_.occupancy_buffer_inflate_.clear();

    for (int i = 0; i < 3; ++i)
      md_.coarse_voxel_num_(i) = ceil(mp_.map_size_(i) * mp_.coarse_resolution_inv_);
    size_t coarse_size = (size_t)md_.coarse_voxel_num_(0) * (size_t)md_.coarse_voxel_num_(1) *
                         (size_t)md_.coarse_voxel_num_(2);
    md_.occupancy_buffer_coarse_ = vector<char>(coarse_size, 0);

    // 窗口半宽 = local_update_range + 最大膨胀 + 1 个 coarse 格（重叠带，保证无缝切换）
    double infl_max = max(mp_.obstacles_inflation_, mp_.obstacles_inflation_z_);
    for (int i = 0; i < 3; ++i)
    {
      double win_half = mp_.local_update_range_(i) + infl_max + mp_.coarse_resolution_;
      md_.fine_n_(i) = (int)ceil(2.0 * win_half * mp_.fine_resolution_inv_) + 1;
    }
    size_t fine_size = (size_t)md_.fine_n_(0) * (size_t)md_.fine_n_(1) * (size_t)md_.fine_n_(2);
    md_.occupancy_buffer_fine_ = vector<char>(fine_size, 0);
    md_.fine_window_init_ = false;

    // 单层语义下 map_voxel_num_ 按 fine 计，仅用于边界钳制，不用于分配整图缓冲
    mp_.fine_resolution_ = mp_.resolution_;
    mp_.fine_resolution_inv_ = mp_.resolution_inv_;
    md_.fine_inf_step_ = max(0, (int)ceil(mp_.obstacles_inflation_ / mp_.fine_resolution_));
    md_.fine_inf_step_z_ = max(0, (int)ceil(mp_.obstacles_inflation_z_ / mp_.fine_resolution_));

    RCLCPP_INFO(node_->get_logger(),
                "[grid_map] 多层级地图: fine %.3fm 窗口 %d x %d x %d (%.0f MB) + coarse %.3fm 全图 %d x %d x %d (%.0f MB)",
                mp_.fine_resolution_, md_.fine_n_(0), md_.fine_n_(1), md_.fine_n_(2),
                fine_size / 1048576.0, mp_.coarse_resolution_,
                md_.coarse_voxel_num_(0), md_.coarse_voxel_num_(1), md_.coarse_voxel_num_(2),
                coarse_size / 1048576.0);
  }
  else
  {
    md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

    // 点云模式下不启用深度射线投射：不分配这几块随全图尺寸增长的大缓冲，避免无谓内存占用
    if (mp_.enable_depth_raycast_)
    {
      md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
      md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
      md_.count_hit_ = vector<short>(buffer_size, 0);
      md_.flag_rayend_ = vector<char>(buffer_size, -1);
      md_.flag_traverse_ = vector<char>(buffer_size, -1);
    }
  }

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  // 初始化 message_filters::Subscriber
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      node_, "grid_map/depth", rclcpp::QoS(50).get_rmw_qos_profile());

  extrinsic_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/vins_estimator/extrinsic", 10,
      std::bind(&GridMap::extrinsicCallback, this, std::placeholders::_1));

  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
        node_, "grid_map/pose", rclcpp::QoS(25).get_rmw_qos_profile());

    sync_image_pose_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImagePose>>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_);
    sync_image_pose_->registerCallback(
        std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
        node_, "grid_map/odom", rclcpp::QoS(100).get_rmw_qos_profile());

    sync_image_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImageOdom>>(
        SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_);
    sync_image_odom_->registerCallback(
        std::bind(&GridMap::depthOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  // 使用独立的里程计和点云订阅
  indep_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "grid_map/cloud", rclcpp::QoS(1).best_effort(), std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));

  indep_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "grid_map/odom", rclcpp::QoS(1).best_effort(), std::bind(&GridMap::odomCallback, this, std::placeholders::_1));

  // 定时器
  occ_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.05),
      std::bind(&GridMap::updateOccupancyCallback, this));

  vis_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.11),
      std::bind(&GridMap::visCallback, this));

  // 发布者
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());

  is_map_updating_ = false;
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

void GridMap::clearOriginSafeZone()
{
  static bool first_call = true;
  if (first_call)
  {
    RCLCPP_DEBUG(node_->get_logger(), 
                "clearOriginSafeZone called. enabled=%d, size=[%.2f, %.2f, %.2f]",
                mp_.origin_safe_zone_enabled_,
                mp_.origin_safe_zone_size_(0),
                mp_.origin_safe_zone_size_(1),
                mp_.origin_safe_zone_size_(2));
    first_call = false;
  }

  if (!mp_.origin_safe_zone_enabled_)
    return;

  Eigen::Vector3d origin(0.0, 0.0, 0.0);
  Eigen::Vector3d min_pos = origin - mp_.origin_safe_zone_size_ / 2.0;
  Eigen::Vector3d max_pos = origin + mp_.origin_safe_zone_size_ / 2.0;

  min_pos(2) = std::max(min_pos(2), mp_.map_min_boundary_(2));
  max_pos(2) = std::min(max_pos(2), mp_.map_max_boundary_(2));

  RCLCPP_DEBUG(node_->get_logger(), 
               "Safe zone range: min=[%.2f, %.2f, %.2f], max=[%.2f, %.2f, %.2f], map_min_z=%.2f, map_max_z=%.2f",
               min_pos(0), min_pos(1), min_pos(2), 
               max_pos(0), max_pos(1), max_pos(2),
               mp_.map_min_boundary_(2), mp_.map_max_boundary_(2));

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  int cleared_count = 0;
  if (mp_.multilevel_)
  {
    // fine：清窗口内落在安全区的占据格
    for (int x = min_id(0); x <= max_id(0); ++x)
      for (int y = min_id(1); y <= max_id(1); ++y)
        for (int z = min_id(2); z <= max_id(2); ++z)
        {
          Eigen::Vector3i id(x, y, z);
          if (!isFineActive(id))
            continue;
          size_t faddr = toFineAddress(id);
          if (md_.occupancy_buffer_fine_[faddr])
          {
            md_.occupancy_buffer_fine_[faddr] = 0;
            cleared_count++;
          }
        }

    // coarse：清安全区覆盖的粗格；有变更才重建可视化脏点缓存（罕见事件，可接受全扫）
    int cr = mp_.coarse_ratio_;
    bool coarse_changed = false;
    int cx1 = min(max_id(0) / cr, md_.coarse_voxel_num_(0) - 1);
    int cy1 = min(max_id(1) / cr, md_.coarse_voxel_num_(1) - 1);
    int cz1 = min(max_id(2) / cr, md_.coarse_voxel_num_(2) - 1);
    for (int cx = min_id(0) / cr; cx <= cx1; ++cx)
      for (int cy = min_id(1) / cr; cy <= cy1; ++cy)
        for (int cz = min_id(2) / cr; cz <= cz1; ++cz)
        {
          size_t addr = toCoarseAddress(Eigen::Vector3i(cx, cy, cz));
          if (md_.occupancy_buffer_coarse_[addr])
          {
            md_.occupancy_buffer_coarse_[addr] = 0;
            coarse_changed = true;
            cleared_count++;
          }
        }
    if (coarse_changed)
      rebuildCoarseDirty();
  }
  else
  {
    for (int x = min_id(0); x <= max_id(0); ++x)
      for (int y = min_id(1); y <= max_id(1); ++y)
        for (int z = min_id(2); z <= max_id(2); ++z)
        {
          size_t idx = toAddress(x, y, z);
          if (md_.occupancy_buffer_inflate_[idx] != 0 ||
              (mp_.enable_depth_raycast_ && md_.occupancy_buffer_[idx] > mp_.clamp_min_log_))
          {
            md_.occupancy_buffer_inflate_[idx] = 0;
            if (mp_.enable_depth_raycast_)
              md_.occupancy_buffer_[idx] = mp_.clamp_min_log_;
            cleared_count++;
          }
        }
  }

  if (cleared_count > 0)
  {
    RCLCPP_DEBUG(node_->get_logger(), 
                "Cleared %d occupied voxels in origin safe zone [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
                cleared_count, min_pos(0), min_pos(1), min_pos(2), max_pos(0), max_pos(1), max_pos(2));
  }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v += skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  rclcpp::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        size_t idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        size_t idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        size_t idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        size_t idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        size_t idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        size_t idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            size_t idx_inf = toAddress(inf_pt);
            if (idx_inf >= md_.occupancy_buffer_inflate_.size())
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.05)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::visCallback()
{
  if (is_map_updating_)
    return;
  publishMapInflate(true);
  publishMap();
}

void GridMap::updateOccupancyCallback()
{
  is_map_updating_ = true;
  if (md_.last_occ_update_time_.seconds() < 1.0)
    md_.last_occ_update_time_ = node_->now();

  if (!md_.occ_need_update_)
  {
    if (md_.flag_use_depth_fusion &&
        (node_->now() - md_.last_occ_update_time_).seconds() > mp_.odom_depth_timeout_)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "odom or depth lost! now=%f, last_occ_update_time=%f, odom_depth_timeout=%f",
                   node_->now().seconds(),
                   md_.last_occ_update_time_.seconds(),
                   mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    clearOriginSafeZone();
    is_map_updating_ = false;
    return;
  }
  md_.last_occ_update_time_ = node_->now();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  if (mp_.enable_depth_raycast_)
  {
    projectDepthImage();
    // t2 = ros::Time::now();
    raycastProcess();
    // t3 = ros::Time::now();

    if (md_.local_updated_)
      clearAndInflateLocalMap();
  }

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  clearOriginSafeZone();
  is_map_updating_ = false;
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img,
                                const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  if (md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

/* ---------------- 多层级地图（cloud 模式） ---------------- */

void GridMap::rebuildCoarseDirty()
{
  // coarse 全图扫一遍重建可视化脏点缓存（仅在初始化和安全区清除后调用）
  md_.coarse_dirty_pts_.clear();
  size_t cn2 = (size_t)md_.coarse_voxel_num_(2);
  for (int x = 0; x < md_.coarse_voxel_num_(0); ++x)
    for (int y = 0; y < md_.coarse_voxel_num_(1); ++y)
    {
      size_t base = ((size_t)x * (size_t)md_.coarse_voxel_num_(1) + (size_t)y) * cn2;
      for (int z = 0; z < md_.coarse_voxel_num_(2); ++z)
      {
        if (!md_.occupancy_buffer_coarse_[base + (size_t)z])
          continue;
        if (md_.coarse_dirty_pts_.size() >= 2000000)
          return;
        md_.coarse_dirty_pts_.push_back(mp_.map_origin_ +
            (Eigen::Vector3d((double)x, (double)y, (double)z) + Eigen::Vector3d(0.5, 0.5, 0.5)) *
            mp_.coarse_resolution_);
      }
    }
}

void GridMap::resetFineWindow(const Eigen::Vector3d &camera_pos)
{
  // 大跳变（起飞/重置）：旧窗口全部写穿进 coarse，再以相机为中心重建并用 coarse 记忆播种
  Eigen::Vector3i g = md_.fine_min_gidx_;
  for (int x0 = 0; x0 < md_.fine_n_(0); ++x0)
  {
    g(0) = md_.fine_min_gidx_(0) + x0;
    for (int y0 = 0; y0 < md_.fine_n_(1); ++y0)
    {
      g(1) = md_.fine_min_gidx_(1) + y0;
      for (int z0 = 0; z0 < md_.fine_n_(2); ++z0)
      {
        g(2) = md_.fine_min_gidx_(2) + z0;
        char v = md_.occupancy_buffer_fine_[toFineAddress(g)];
        if (v)
          markCoarseFromFineG(g, v);
      }
    }
  }

  posToFineIndex(camera_pos, md_.fine_min_gidx_);
  md_.fine_min_gidx_ -= md_.fine_n_ / 2;
  for (int slot = 0; slot < md_.fine_n_(0); ++slot)
  {
    Eigen::Vector3i g_front = md_.fine_min_gidx_;
    g_front(0) += slot;
    upsampleCoarseToFinePlane(0, slot, g_front);
  }
  md_.fine_window_init_ = true;
}

void GridMap::rollFineToCamera(const Eigen::Vector3d &camera_pos)
{
  Eigen::Vector3i cam_g;
  posToFineIndex(camera_pos, cam_g);

  if (!md_.fine_window_init_)
  {
    // 首帧：缓冲刚分配全 0，coarse 也为空，直接定心即可
    md_.fine_min_gidx_ = cam_g - md_.fine_n_ / 2;
    md_.fine_window_init_ = true;
    return;
  }

  Eigen::Vector3i desired = cam_g - md_.fine_n_ / 2;
  bool teleport = false;
  for (int axis = 0; axis < 3; ++axis)
    if (abs(desired(axis) - md_.fine_min_gidx_(axis)) >= md_.fine_n_(axis) / 2)
      teleport = true;

  if (teleport)
  {
    resetFineWindow(camera_pos);
    return;
  }

  for (int axis = 0; axis < 3; ++axis)
  {
    int delta = desired(axis) - md_.fine_min_gidx_(axis);
    if (delta != 0)
      rollFineAxis(axis, delta);
  }
}

void GridMap::rollFineAxis(int axis, int delta)
{
  // 环形绝对寻址下窗口平移 = min 逐格推进：每次滚 1 层，
  // 被挤出的那一层先写穿 coarse，腾出的同槽位再用 coarse 记忆播种新前沿层
  int n = md_.fine_n_(axis);
  int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
  bool dir_pos = delta > 0;
  int steps = dir_pos ? delta : -delta;

  for (int k = 0; k < steps; ++k)
  {
    int g_discard = dir_pos ? md_.fine_min_gidx_(axis) : md_.fine_min_gidx_(axis) + n - 1;
    int slot = (int)((size_t)g_discard % (size_t)n);

    for (int v1 = 0; v1 < md_.fine_n_(a1); ++v1)
      for (int v2 = 0; v2 < md_.fine_n_(a2); ++v2)
      {
        Eigen::Vector3i g = md_.fine_min_gidx_;
        g(axis) = g_discard;
        g(a1) += v1;
        g(a2) += v2;
        char v = md_.occupancy_buffer_fine_[toFineAddress(g)];
        if (v)
          markCoarseFromFineG(g, v);
      }

    md_.fine_min_gidx_(axis) += dir_pos ? 1 : -1;
    Eigen::Vector3i g_front = md_.fine_min_gidx_;
    if (dir_pos)
      g_front(axis) += n - 1;
    upsampleCoarseToFinePlane(axis, slot, g_front);
  }
}

void GridMap::markInflatedPointFine(const Eigen::Vector3d &p3d)
{
  Eigen::Vector3i g0;
  posToFineIndex(p3d, g0);
  int st = md_.fine_inf_step_, stz = md_.fine_inf_step_z_;
  for (int dx = -st; dx <= st; ++dx)
    for (int dy = -st; dy <= st; ++dy)
      for (int dz = -stz; dz <= stz; ++dz)
      {
        Eigen::Vector3i g(g0(0) + dx, g0(1) + dy, g0(2) + dz);
        if (isFineActive(g))
        {
          // 只升级不降级：膨胀格不能覆盖已经标记的原始命中格
          char &cell = md_.occupancy_buffer_fine_[toFineAddress(g)];
          if (cell == OCC_FREE)
            cell = OCC_INFLATE;
        }
        else
          markCoarseFromFineG(g, OCC_INFLATE);  // 越过窗口裕量的罕见情况兜底写 coarse，绝不让障碍丢失
      }

  // 命中点本身记为原始障碍：脱困窗口内只把这一类当障碍
  if (isFineActive(g0))
  {
    char &cell = md_.occupancy_buffer_fine_[toFineAddress(g0)];
    if (cell < OCC_RAW)
      cell = OCC_RAW;
  }
  else
    markCoarseFromFineG(g0, OCC_RAW);
}

void GridMap::markInflatedPointCoarse(const Eigen::Vector3d &p3d)
{
  if (!isInMap(p3d))
    return;
  Eigen::Vector3i c;
  posToCoarseIndex(p3d, c);
  setCoarseCell(c, OCC_RAW);  // 体素内有点云即占据，coarse 层不膨胀
}

void GridMap::beginEscapeWindow(const Eigen::Vector3d &center, double half_size)
{
  escape_active_ = true;
  escape_center_ = center;
  escape_half_ = half_size > 0.0 ? half_size : 0.0;
  RCLCPP_WARN(node_->get_logger(),
              "[grid_map] 脱困窗口开启: center=(%.2f, %.2f, %.2f), 边长 %.2f m（窗口内忽略膨胀层）",
              center(0), center(1), center(2), 2.0 * escape_half_);
}

void GridMap::endEscapeWindow()
{
  if (!escape_active_)
    return;
  escape_active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[grid_map] 脱困窗口关闭，恢复膨胀层判定");
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &img)
{
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  
  // 获取点云的时间戳
  auto cloud_time = img->header.stamp;
  
  try
  {
    pcl::fromROSMsg(*img, latest_cloud);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "点云数据转换失败: %s", e.what());
    return;
  }

  md_.has_cloud_ = true;

  // 近距回波过滤：gz-sim 7.9.0 的 gpu_lidar 在扫描网格的边界行/列（方位 ±180°、俯仰 max_angle）
  // 会取到无效深度 0，并把小于 <range><min> 的距离钳制到 min 量程，于是每帧都出现一圈
  // 半径恰好等于 min 的假点。该假环随传感器移动、在仿真世界中无对应物体，因本机 gz 版本
  // 未给 Lidar 接线 visibility_mask 而无法在模型侧屏蔽，只能在消费端剔除。
  // 注意：阈值 min_ray_length 必须大于雷达 SDF 里的 <range><min>。
  std::string source_frame = img->header.frame_id;
  const bool cloud_in_world = (source_frame == mp_.frame_id_ || source_frame == "map" ||
                               source_frame == "world" || source_frame == "odom");
  if (!cloud_in_world && mp_.min_ray_length_ > 0.0)
  {
    size_t write = 0;
    for (size_t i = 0; i < latest_cloud.points.size(); ++i)
    {
      const auto &p = latest_cloud.points[i];
      const double d = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      if (!std::isfinite(d) || d < mp_.min_ray_length_)
        continue;
      latest_cloud.points[write++] = p;
    }
    // resize() 会同步 width/height；若只改 points.size()，后续 pcl::transformPointCloud
    // 会按旧的 width(=水平线数 360) 重新 assign，点数除不尽时报 "Mismatch in assignment"
    latest_cloud.resize(write);
  }

  if (!md_.has_odom_)
  {
    RCLCPP_WARN(node_->get_logger(), "点云数据接收但odom数据缺失");
    return;
  }

  if (latest_cloud.points.size() == 0)
  {
    static int count_zero_or_rez_ = 0;
    count_zero_or_rez_++;
    if (count_zero_or_rez_ % 100 == 0)
    {
      RCLCPP_WARN(node_->get_logger(), "点云数据点数为0, 第%d次", count_zero_or_rez_);
      count_zero_or_rez_ = 1;
    }
    return;
  }

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
  {
    RCLCPP_WARN(node_->get_logger(), "相机位置数据无效");
    return;
  }
  
  // 将点云从相机坐标系转换到世界坐标系
  pcl::PointCloud<pcl::PointXYZ> world_cloud;
  
  // 检查点云是否已经在world frame
  static bool first_time = true;
  if (cloud_in_world)
  {
    if (first_time)
    {
      first_time = false;
      // 点云已经在world frame，不需要转换
      RCLCPP_INFO(node_->get_logger(), "点云已经在%s坐标系，跳过转换，点数: %zu", 
                 mp_.frame_id_.c_str(), latest_cloud.points.size());
    }
  }
  else
  {
    if (first_time)
    {
      first_time = false;
      RCLCPP_INFO(node_->get_logger(), "尝试将点云从%s转换到%s坐标系，点数: %zu", 
                 source_frame.c_str(), mp_.frame_id_.c_str(), latest_cloud.points.size());
    }    
    try
    {
      // 使用全局TF2缓冲区（避免重复创建）
      if (!tf_buffer_) {
        RCLCPP_ERROR(node_->get_logger(), "TF2缓冲区未初始化，无法进行坐标系转换");
        return;
      }
      
      // 获取从相机坐标系到世界坐标系的变换
      rclcpp::Duration timeout(-500000000, 500000000); // 500毫秒超时
      geometry_msgs::msg::TransformStamped transform;
      
      if (source_frame.empty())
      {
        source_frame = "x500_depth_0/StereoOV7251"; // 默认相机坐标系
      }
      
      transform = tf_buffer_->lookupTransform(mp_.frame_id_, source_frame, cloud_time, timeout);
      
      // 获取变换矩阵
      Eigen::Isometry3d transform_matrix = tf2::transformToEigen(transform);
      
      // 预分配内存避免重复分配
      world_cloud.points.resize(latest_cloud.points.size());
      
      // 使用PCL的高效转换函数（需要将Eigen::Isometry3d转换为Eigen::Affine3d）
      Eigen::Affine3d affine_transform(transform_matrix);
      pcl::transformPointCloud(latest_cloud, world_cloud, affine_transform);
      
      RCLCPP_DEBUG(node_->get_logger(), "成功将点云从%s转换到%s坐标系，点数: %zu", 
                   source_frame.c_str(), mp_.frame_id_.c_str(), world_cloud.points.size());
      
      // 使用转换后的世界坐标系点云
      latest_cloud = world_cloud;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "TF变换失败: %s，无法处理点云数据", ex.what());
      return;
    }
  }
  
  // 小范围减：复用 RayCaster 做射线雕刻清除。
  // 对本帧每个回波点，从相机向命中点打射线，只清除落在 clear 盒内的沿途栅格，
  // 即「本帧回波确认是自由空间的地方」才清空；射线打不到、无回波确认的区域保留历史记忆。
  // 远处的点同样参与雕刻（射线截断到 max_ray_length_），用远端命中清理近端射线。
  // 后面的标记循环为第二遍，命中点会被重新置 1，保证「击中赢」。
  // 多层级：每次点云处理前先把 fine 窗口平移到相机位置（滚出部分写穿 coarse，滚入部分用 coarse 播种）
  if (mp_.multilevel_)
    rollFineToCamera(md_.camera_pos_);

  if (mp_.multilevel_ || mp_.reset_buffer_enabled_)
  {
    RayCaster raycaster;
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d ray_pt, pt_w;

    // clear 盒的索引边界（与 resetBuffer 相同的取整+钳制方式）
    Eigen::Vector3i clear_min_id, clear_max_id;
    posToIndex(md_.camera_pos_ - mp_.clear_range_, clear_min_id);
    posToIndex(md_.camera_pos_ + mp_.clear_range_, clear_max_id);
    boundIndex(clear_min_id);
    boundIndex(clear_max_id);

    for (size_t i = 0; i < latest_cloud.points.size(); ++i)
    {
      const pcl::PointXYZ &cpt = latest_cloud.points[i];
      if (!std::isfinite(cpt.x) || !std::isfinite(cpt.y) || !std::isfinite(cpt.z))
        continue;
      pt_w(0) = cpt.x, pt_w(1) = cpt.y, pt_w(2) = cpt.z;

      // 越界点截断到地图边界，超长射线截断到 max_ray_length_（与 raycastProcess 一致）
      if (!isInMap(pt_w))
        pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      double ray_len = (pt_w - md_.camera_pos_).norm();
      if (ray_len > mp_.max_ray_length_)
        pt_w = (pt_w - md_.camera_pos_) / ray_len * mp_.max_ray_length_ + md_.camera_pos_;

      // 方向必须为 相机→命中点：step() 不输出 end 格，天然不清命中格本身（由后面的标记遍置 1），
      // 且射线从盒中心单调向外，出盒 break 前已清完盒内近端格；远处命中点也能清理近端射线
      raycaster.setInput(md_.camera_pos_ / mp_.resolution_, pt_w / mp_.resolution_);

      while (raycaster.step(ray_pt))
      {
        Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;

        Eigen::Vector3i id;
        posToIndex(tmp, id);

        // 相机位于 clear 盒中心，射线单调向外：一旦出盒即可终止
        if (id(0) < clear_min_id(0) || id(0) > clear_max_id(0) ||
            id(1) < clear_min_id(1) || id(1) > clear_max_id(1) ||
            id(2) < clear_min_id(2) || id(2) > clear_max_id(2))
          break;

        // 写 0 幂等，无需帧戳去重。raycast 消减只作用于 fine 层；coarse 层持久不清
        if (mp_.multilevel_)
        {
          if (isFineActive(id))
            md_.occupancy_buffer_fine_[toFineAddress(id)] = 0;
        }
        else
          md_.occupancy_buffer_inflate_[toAddress(id)] = 0;
      }
    }
  }

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  // 横向/纵向各自量化到整格：横向对应桨叶扫掠半径，纵向对应机体半高，两者互不牵连。
  // 下限取 0（inflation 配成 0 表示只占自身体素），避免负数 step 让膨胀圈整个静默失效
  int inf_step = max(0, (int)ceil(mp_.obstacles_inflation_ / mp_.resolution_));
  int inf_step_z = max(0, (int)ceil(mp_.obstacles_inflation_z_ / mp_.resolution_));

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  is_map_updating_ = true;
  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* 加：落在 local_update_range 内的点才标记，超出范围的旧标记保留（记忆） */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2))
    {
      if (mp_.multilevel_)
      {
        // fine 层膨胀标记（步数用 fine_inf_step_，出窗兜底写穿 coarse），只更新包围盒
        markInflatedPointFine(p3d);
        double inf_xy = mp_.obstacles_inflation_;
        double inf_z = mp_.obstacles_inflation_z_;
        max_x = max(max_x, p3d(0) + inf_xy);
        max_y = max(max_y, p3d(1) + inf_xy);
        max_z = max(max_z, p3d(2) + inf_z);
        min_x = min(min_x, p3d(0) - inf_xy);
        min_y = min(min_y, p3d(1) - inf_xy);
        min_z = min(min_z, p3d(2) - inf_z);
      }
      else
      {

        /* inflate the point */
        // 点云膨胀
        for (int x = -inf_step; x <= inf_step; ++x)
          for (int y = -inf_step; y <= inf_step; ++y)
            for (int z = -inf_step_z; z <= inf_step_z; ++z)
            {

              p3d_inf(0) = pt.x + x * mp_.resolution_;
              p3d_inf(1) = pt.y + y * mp_.resolution_;
              p3d_inf(2) = pt.z + z * mp_.resolution_;

              max_x = max(max_x, p3d_inf(0));
              max_y = max(max_y, p3d_inf(1));
              max_z = max(max_z, p3d_inf(2));

              min_x = min(min_x, p3d_inf(0));
              min_y = min(min_y, p3d_inf(1));
              min_z = min(min_z, p3d_inf(2));

              posToIndex(p3d_inf, inf_pt);

              if (!isInMap(inf_pt))
                continue;

              size_t idx_inf = toAddress(inf_pt);

              md_.occupancy_buffer_inflate_[idx_inf] = 1;
            }
      }
    }
    else if (mp_.multilevel_)
    {
      // local_update_range 外的点：直接命中 coarse 格（体素内有点即占据，不膨胀）
      markInflatedPointCoarse(p3d);
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  // 更新局部地图边界
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // add virtual ceiling to limit flight height
  // 添加虚拟天花板控制飞行高度：向下铺 inf_step_z 格的禁飞带（厚度 >= obstacles_inflation_z），
  // 原来只铺 1 格，等于机体上半高没算进天花板余量
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = min(mp_.map_voxel_num_(2) - 1,
                      (int)floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1);
    int ceil_bottom_id = max(0, ceil_id - max(1, inf_step_z) + 1);
    // RCLCPP_INFO(node_->get_logger(), "添加虚拟天花板: 高度=%.2f, 栅格ID=%d", mp_.virtual_ceil_height_, ceil_id);
    if (mp_.multilevel_)
    {
      // fine：窗口滚入新层时由 coarse 播种，天花板不在 coarse 里，必须每帧铺满整个窗口带
      Eigen::Vector3i gmin = md_.fine_min_gidx_;
      Eigen::Vector3i gmax = md_.fine_min_gidx_ + md_.fine_n_ - Eigen::Vector3i::Ones();
      int x0 = max(gmin(0), 0), x1 = min(gmax(0), mp_.map_voxel_num_(0) - 1);
      int y0 = max(gmin(1), 0), y1 = min(gmax(1), mp_.map_voxel_num_(1) - 1);
      for (int x = x0; x <= x1; ++x)
        for (int y = y0; y <= y1; ++y)
          for (int z = ceil_bottom_id; z <= ceil_id; ++z)
            md_.occupancy_buffer_fine_[toFineAddress(Eigen::Vector3i(x, y, z))] = OCC_RAW;

      // coarse：只铺本帧包围盒覆盖的粗格一层（远场无规划，兜底查询够用）
      int cr = mp_.coarse_ratio_;
      int cceil = min(md_.coarse_voxel_num_(2) - 1,
                      (int)floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.coarse_resolution_inv_) - 1);
      if (cceil >= 0)
        for (int cx = md_.local_bound_min_(0) / cr; cx <= md_.local_bound_max_(0) / cr; ++cx)
          for (int cy = md_.local_bound_min_(1) / cr; cy <= md_.local_bound_max_(1) / cr; ++cy)
          {
            Eigen::Vector3i c(cx, cy, cceil);
            boundCoarseIndex(c);
            setCoarseCell(c, OCC_RAW);
          }
    }
    else
    {
      for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
          for (int z = ceil_bottom_id; z <= ceil_id; ++z) {
            md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 1;
          }
    }
  }
  clearOriginSafeZone();
  is_map_updating_ = false;
}

void GridMap::publishMap()
{
  // 点云模式下未分配 occupancy_buffer_，且该话题无订阅者，直接跳过
  if (!mp_.enable_depth_raycast_ || md_.occupancy_buffer_.empty())
  {
    return;
  }

  if (map_pub_->get_subscription_count() <= 0)
  {
    return;
  }

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;  // 点云已经在世界坐标系中
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info){

  if (map_inf_pub_->get_subscription_count()<= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // 显示窗口 = 本帧点云包围盒(local_bound) 与 camera±local_update_range 滚动窗口的并集。
  // 只用包围盒时，贴墙等点云收缩场景会把未删除的历史记忆格"隐藏"在话题之外；
  // 并入滚动窗口后，RViz 显示范围与地图记忆/更新范围一致，纯显示层改动。
  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  Eigen::Vector3i win_min, win_max;
  posToIndex(md_.camera_pos_ - mp_.local_update_range_, win_min);
  posToIndex(md_.camera_pos_ + mp_.local_update_range_, win_max);
  min_cut = min_cut.cwiseMin(win_min);
  max_cut = max_cut.cwiseMax(win_max);

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  if (mp_.multilevel_)
  {
    // fine：显示窗口(min_cut/max_cut 已是 fine 全局索引)与滚动窗口求交后扫描
    Eigen::Vector3i gmin = md_.fine_min_gidx_;
    Eigen::Vector3i gmax = md_.fine_min_gidx_ + md_.fine_n_ - Eigen::Vector3i::Ones();
    Eigen::Vector3i smin = min_cut.cwiseMax(gmin);
    Eigen::Vector3i smax = max_cut.cwiseMin(gmax);
    for (int x = smin(0); x <= smax(0); ++x)
      for (int y = smin(1); y <= smax(1); ++y)
        for (int z = smin(2); z <= smax(2); ++z)
        {
          Eigen::Vector3i g(x, y, z);
          if (md_.occupancy_buffer_fine_[toFineAddress(g)] == 0)
            continue;

          Eigen::Vector3d pos;
          indexToPos(g, pos);
          if (pos(2) > mp_.visualization_truncate_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }

    // coarse：直接发布增量脏点缓存（持久记忆区，滚出窗口的历史占据）
    for (size_t i = 0; i < md_.coarse_dirty_pts_.size(); ++i)
    {
      const Eigen::Vector3d &p = md_.coarse_dirty_pts_[i];
      if (p(2) > mp_.visualization_truncate_height_)
        continue;
      pt.x = p(0);
      pt.y = p(1);
      pt.z = p(2);
      cloud.push_back(pt);
    }
  }
  else
  {
    for (int x = min_cut(0); x <= max_cut(0); ++x)
      for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_->publish(cloud_msg);

  // RCLCPP_INFO(rclcpp::get_logger("publishMapInflate"), "pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img,
                                const nav_msgs::msg::Odometry::ConstSharedPtr &odom)
{
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}