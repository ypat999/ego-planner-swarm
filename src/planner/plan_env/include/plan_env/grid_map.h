#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

// 前向声明TF2相关类
namespace tf2_ros {
class Buffer;
class TransformListener;
}

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const &matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters
{

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map range in pos
  Eigen::Vector3i map_voxel_num_;                       // map range in index
  Eigen::Vector3d local_update_range_;
  /* 点云模式：加(mark)复用 local_update_range_，减(clear)用小范围以兼顾近端动态 */
  Eigen::Vector3d clear_range_;
  double resolution_, resolution_inv_;
  /* 多层级（cloud 模式）：resolution_ 即精细层分辨率；coarse 层为整数倍更粗分辨率 */
  double fine_resolution_, fine_resolution_inv_;
  double coarse_resolution_, coarse_resolution_inv_;
  int coarse_ratio_;
  double obstacles_inflation_;
  /* z 方向独立膨胀半径（对应机体半高）；横向用 obstacles_inflation_（对应桨叶半径）。
     两者都会按 ceil(inflation / resolution) 量化到整格，所以不要指望配出的值精确相等 */
  double obstacles_inflation_z_;
  string frame_id_;
  int pose_type_;

  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* time out */
  double odom_depth_timeout_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_depth_filter_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                  // logit of occupancy probability
  double min_ray_length_, max_ray_length_; // range of doing raycasting

  /* local map update and clear */
  int local_map_margin_;

  /* visualization and computation time display */
  double visualization_truncate_height_, virtual_ceil_height_, ground_height_, virtual_ceil_yp_, virtual_ceil_yn_;
  bool show_occ_time_;

  /* active mapping */
  double unknown_flag_;

  /* buffer reset control */
  bool reset_buffer_enabled_;

  /* depth raycast buffers (disable in cloud mode to save memory) */
  bool enable_depth_raycast_;

  /* cloud 模式多层级地图：fine 环形滚动窗口(local_update_range 内高分辨率) + coarse 全局记忆层 */
  bool multilevel_;

  /* origin safe zone for landing */
  bool origin_safe_zone_enabled_;
  Eigen::Vector3d origin_safe_zone_size_;
};

// intermediate mapping data for fusion

struct MappingData
{
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;

  // 多层级（cloud 模式）：粗糙全局层 + 精细环形窗口层
  std::vector<char> occupancy_buffer_coarse_;
  Eigen::Vector3i coarse_voxel_num_;
  std::vector<char> occupancy_buffer_fine_;
  Eigen::Vector3i fine_n_;         // 精细环形缓冲每轴体素数
  Eigen::Vector3i fine_min_gidx_;  // 精细窗口最小角（全局精细索引）
  int fine_inf_step_, fine_inf_step_z_;
  bool fine_window_init_;
  std::vector<Eigen::Vector3d> coarse_dirty_pts_;  // coarse 0->1 增量记录（仅可视化用）

  // camera position and pose data

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Matrix3d camera_r_m_, last_camera_r_m_;
  Eigen::Matrix4d cam2body_;

  // depth image data

  cv::Mat depth_image_, last_depth_image_;
  int image_cnt_;

  // flags of map state

  bool occ_need_update_, local_updated_;
  bool has_first_depth_;
  bool has_odom_, has_cloud_;

  // odom_depth_timeout_
  rclcpp::Time last_occ_update_time_;
  bool flag_depth_odom_timeout_;
  bool flag_use_depth_fusion;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  GridMap() : tf_buffer_(nullptr), tf_listener_(nullptr) {}
  ~GridMap() {}

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);
  void clearOriginSafeZone();

  inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  inline void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  // 返回 size_t：地图体素数可能超过 INT_MAX（如 400m×400m×20m @0.1m ≈ 3.2e9），
  // 用 int 计算地址会溢出成负数，导致 vector 长度非法直接 abort
  inline size_t toAddress(const Eigen::Vector3i &id);
  inline size_t toAddress(int &x, int &y, int &z);
  inline bool isInMap(const Eigen::Vector3d &pos);
  inline bool isInMap(const Eigen::Vector3i &idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos);

  inline void boundIndex(Eigen::Vector3i &id);
  inline bool isUnknown(const Eigen::Vector3i &id);
  inline bool isUnknown(const Eigen::Vector3d &pos);
  inline bool isKnownFree(const Eigen::Vector3i &id);
  inline bool isKnownOccupied(const Eigen::Vector3i &id);

  void initMap(rclcpp::Node::SharedPtr node);

  void publishMap();
  void publishMapInflate(bool all_info = false);

  void publishDepth();

  bool hasDepthObservation();
  bool hasCloudObservation();
  bool odomValid();
  void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
  inline double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();
  bool getOdomDepthTimeout() { return md_.flag_depth_odom_timeout_; }

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;

  // get depth image and camera pose
  void depthPoseCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                         const geometry_msgs::msg::PoseStamped::ConstPtr &pose);
  void extrinsicCallback(const nav_msgs::msg::Odometry::ConstPtr &odom);
  void depthOdomCallback(const sensor_msgs::msg::Image::ConstPtr &img, const nav_msgs::msg::Odometry::ConstPtr &odom);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &img);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  // update occupancy by raycasting
  void updateOccupancyCallback();
  void visCallback();

  // main update process
  void projectDepthImage();
  void raycastProcess();
  void clearAndInflateLocalMap();

  // 多层级（cloud 模式）
  void rollFineToCamera(const Eigen::Vector3d &camera_pos);
  void rollFineAxis(int axis, int delta);
  void resetFineWindow(const Eigen::Vector3d &camera_pos);
  void rebuildCoarseDirty();
  void markInflatedPointFine(const Eigen::Vector3d &p3d);
  void markInflatedPointCoarse(const Eigen::Vector3d &p3d);
  inline void setCoarseCell(const Eigen::Vector3i &c);
  inline void markCoarseFromFineG(const Eigen::Vector3i &g);
  inline void upsampleCoarseToFinePlane(int axis, int slot, const Eigen::Vector3i &g_front);
  inline void posToFineIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &g);
  inline void posToCoarseIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &c);
  inline bool isFineActive(const Eigen::Vector3i &g);
  inline size_t toFineAddress(const Eigen::Vector3i &g);
  inline size_t toCoarseAddress(const Eigen::Vector3i &c);
  inline void boundCoarseIndex(Eigen::Vector3i &c);

  inline void inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts);
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, nav_msgs::msg::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>
      SyncPolicyImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> pose_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr indep_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr indep_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr extrinsic_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_inf_pub_;

  rclcpp::TimerBase::SharedPtr occ_timer_;
  rclcpp::TimerBase::SharedPtr vis_timer_;

  // TF2缓冲区缓存（全局变量，避免重复创建）
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;

  bool is_map_updating_;
};

/* ============================== definition of inline function
 * ============================== */

inline size_t GridMap::toAddress(const Eigen::Vector3i &id)
{
  return (size_t)id(0) * (size_t)mp_.map_voxel_num_(1) * (size_t)mp_.map_voxel_num_(2) +
         (size_t)id(1) * (size_t)mp_.map_voxel_num_(2) + (size_t)id(2);
}

inline size_t GridMap::toAddress(int &x, int &y, int &z)
{
  return (size_t)x * (size_t)mp_.map_voxel_num_(1) * (size_t)mp_.map_voxel_num_(2) +
         (size_t)y * (size_t)mp_.map_voxel_num_(2) + (size_t)z;
}

inline void GridMap::boundIndex(Eigen::Vector3i &id)
{
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  size_t adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  size_t adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_inflate_[toAddress(id)] = 1;
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ)
{
  if (occ != 1 && occ != 0)
  {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ;
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  if (!mp_.enable_depth_raycast_)
  {
    // cloud 模式：精细窗口优先，否则粗糙全局层
    Eigen::Vector3i g;
    posToFineIndex(pos, g);
    if (isFineActive(g))
      return int(md_.occupancy_buffer_fine_[toFineAddress(g)]);

    Eigen::Vector3i c;
    posToCoarseIndex(pos, c);
    boundCoarseIndex(c);
    return int(md_.occupancy_buffer_coarse_[toCoarseAddress(c)]);
  }

  // depth 模式：单层全局栅格
  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline int GridMap::getOccupancy(Eigen::Vector3i id)
{
  if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
      id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
{
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4)
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i &idx)
{
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
  {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1)
  {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline void GridMap::inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts)
{
  int num = 0;
  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z)
      {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}

inline void GridMap::posToFineIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &g)
{
  for (int i = 0; i < 3; ++i)
    g(i) = (int)floor((pos(i) - mp_.map_origin_(i)) * mp_.fine_resolution_inv_);
}

inline void GridMap::posToCoarseIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &c)
{
  for (int i = 0; i < 3; ++i)
    c(i) = (int)floor((pos(i) - mp_.map_origin_(i)) * mp_.coarse_resolution_inv_);
}

inline bool GridMap::isFineActive(const Eigen::Vector3i &g)
{
  return g(0) >= md_.fine_min_gidx_(0) && g(0) < md_.fine_min_gidx_(0) + md_.fine_n_(0) &&
         g(1) >= md_.fine_min_gidx_(1) && g(1) < md_.fine_min_gidx_(1) + md_.fine_n_(1) &&
         g(2) >= md_.fine_min_gidx_(2) && g(2) < md_.fine_min_gidx_(2) + md_.fine_n_(2);
}

inline size_t GridMap::toFineAddress(const Eigen::Vector3i &g)
{
  // 绝对取模环形寻址：slot = g mod n，窗口 min 平移时无需搬迁数据。
  // int 转 size_t 后取模对负数同样得到正确的环绕槽位
  size_t s0 = (size_t)g(0) % (size_t)md_.fine_n_(0);
  size_t s1 = (size_t)g(1) % (size_t)md_.fine_n_(1);
  size_t s2 = (size_t)g(2) % (size_t)md_.fine_n_(2);
  return (s0 * (size_t)md_.fine_n_(1) + s1) * (size_t)md_.fine_n_(2) + s2;
}

inline size_t GridMap::toCoarseAddress(const Eigen::Vector3i &c)
{
  return ((size_t)c(0) * (size_t)md_.coarse_voxel_num_(1) + (size_t)c(1)) * (size_t)md_.coarse_voxel_num_(2) +
         (size_t)c(2);
}

inline void GridMap::boundCoarseIndex(Eigen::Vector3i &c)
{
  c(0) = max(min(c(0), md_.coarse_voxel_num_(0) - 1), 0);
  c(1) = max(min(c(1), md_.coarse_voxel_num_(1) - 1), 0);
  c(2) = max(min(c(2), md_.coarse_voxel_num_(2) - 1), 0);
}

// coarse 格置占据；0->1 时记录可视化脏点（有上限，超限后靠全量重建兜底）
inline void GridMap::setCoarseCell(const Eigen::Vector3i &c)
{
  if (c(0) < 0 || c(1) < 0 || c(2) < 0 ||
      c(0) >= md_.coarse_voxel_num_(0) || c(1) >= md_.coarse_voxel_num_(1) ||
      c(2) >= md_.coarse_voxel_num_(2))
    return;
  size_t addr = toCoarseAddress(c);
  if (md_.occupancy_buffer_coarse_[addr] != 0)
    return;
  md_.occupancy_buffer_coarse_[addr] = 1;
  if (md_.coarse_dirty_pts_.size() < 2000000)
  {
    Eigen::Vector3d pos = mp_.map_origin_ +
                          (c.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * mp_.coarse_resolution_;
    md_.coarse_dirty_pts_.push_back(pos);
  }
}

// fine 全局索引 -> coarse 占据（用于 fine 滚出窗口时的写穿）
inline void GridMap::markCoarseFromFineG(const Eigen::Vector3i &g)
{
  Eigen::Vector3i c;
  for (int i = 0; i < 3; ++i)
    c(i) = (int)floor((double)g(i) / mp_.coarse_ratio_);
  setCoarseCell(c);
}

// fine 窗口滚入新区域时，用 coarse 记忆播种该平面：coarse 占据处 fine 也必须占据，
// 否则查询走 fine 分支会把 coarse 里的旧障碍当作自由空间（安全性回归）
inline void GridMap::upsampleCoarseToFinePlane(int axis, int slot, const Eigen::Vector3i &g_front)
{
  int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
  for (int v1 = 0; v1 < md_.fine_n_(a1); ++v1)
    for (int v2 = 0; v2 < md_.fine_n_(a2); ++v2)
    {
      Eigen::Vector3i g = g_front;
      g(a1) += v1;
      g(a2) += v2;
      Eigen::Vector3i c;
      for (int i = 0; i < 3; ++i)
        c(i) = (int)floor((double)g(i) / mp_.coarse_ratio_);
      char val = 0;
      if (c(0) >= 0 && c(1) >= 0 && c(2) >= 0 &&
          c(0) < md_.coarse_voxel_num_(0) && c(1) < md_.coarse_voxel_num_(1) &&
          c(2) < md_.coarse_voxel_num_(2))
        val = md_.occupancy_buffer_coarse_[toCoarseAddress(c)];

      Eigen::Vector3i s;
      s(axis) = slot;
      s(a1) = (size_t)g(a1) % (size_t)md_.fine_n_(a1);
      s(a2) = (size_t)g(a2) % (size_t)md_.fine_n_(a2);
      md_.occupancy_buffer_fine_[(s(0) * (size_t)md_.fine_n_(1) + s(1)) * (size_t)md_.fine_n_(2) + s(2)] = val;
    }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif