#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <random>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <thread>
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
  /* cloud 自过滤半径(米): 与传感器距离小于该值的回波视为机身/支架自反射，直接丢弃；
     0 表示不过滤。不加此过滤时自反射点会被膨胀标记，机身周围常驻虚假占据，
     多层级下还会经滚出写穿永久进入 coarse 记忆 */
  double cloud_self_filter_radius_;
  /* 点云"只处理最新帧"阈值(秒)：本帧时间戳落后当前 ROS 时钟超过该值，说明回调已被点云
     积压拖住（单帧耗时 > 周期），此时整帧丢弃、把执行器线程让给 odom 回调；<=0 关闭。
     代价：地图更新频率上限被压到 1/耗时，但每帧都是当时能拿到的最新观测 */
  double cloud_stale_max_age_;
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
  // coarse 每字节 = 一个粗格的占据状态（0 空 / OCC_RAW 占据），内存只由 coarse 分辨率决定，
  // 与 fine 分辨率无关；只由 fine 窗口按 coarse 对齐滚出/重建时提交
  std::vector<char> occupancy_buffer_coarse_;
  Eigen::Vector3i coarse_voxel_num_;
  std::vector<char> occupancy_buffer_fine_;
  Eigen::Vector3i fine_n_;         // 精细环形缓冲每轴体素数
  Eigen::Vector3i fine_min_gidx_;  // 精细窗口最小角（全局精细索引）
  int fine_inf_step_, fine_inf_step_z_;
  bool fine_window_init_;
  bool fine_window_dirty_;  // fine 窗口本帧被铺/滚/重建，虚拟天花板细带需全窗口重铺一次
  std::vector<Eigen::Vector3d> coarse_dirty_pts_;  // coarse 0->非0 增量记录（仅可视化用，发布按当前位图过滤）

  // camera position and pose data

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  /* 多层级下 camera_pos_ 归 cloud worker 独占（本帧地图的唯一基准位姿），
     odomCallback 只把最新里程计位姿写到这里，由 executor 侧转发时加锁取用 */
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
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
  ~GridMap() { stopCloudWorker(); }

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };

  // 占据缓冲三态编码（cloud 模式 fine/coarse 共用）：
  // 0=自由，1=仅膨胀（膨胀盒内、非命中格），2=原始命中（同时也属于膨胀体）。
  // 脱困窗口内只把 2 当作障碍，窗口外仍把 >=1 当作障碍。
  enum OccupancyState
  {
    OCC_FREE = 0,
    OCC_INFLATE = 1,
    OCC_RAW = 2
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
  // 会受脱困窗口影响：窗口内按原始命中格判定，窗口外按膨胀层判定
  inline int getInflateOccupancy(Eigen::Vector3d pos);
  // 始终按膨胀层判定（用于判断机体是否真的还在膨胀区里）
  inline int getInflateOccupancyNoEscape(Eigen::Vector3d pos);
  // 只认原始命中格（缓冲值 == OCC_RAW）；depth 模式没有原始层，退回膨胀层
  inline int getRawOccupancy(Eigen::Vector3d pos);

  /* 脱困窗口：以 center 为心、half_size 为半边的立方体内临时只认原始障碍，
     为规划器留出空间；窗口外一切照旧。开关由 FSM 控制。 */
  void beginEscapeWindow(const Eigen::Vector3d &center, double half_size);
  void endEscapeWindow();
  inline bool escapeWindowActive() const { return escape_active_; }
  inline bool inEscapeWindow(const Eigen::Vector3d &pos) const;

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
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &img);
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
  // 以全局 fine 索引为中心的膨胀标记；内部按槽位地址增量推进，不做逐格取模
  void markInflatedPointFine(const Eigen::Vector3i &g0);
  // fine -> coarse 的唯一写入口：整粗格提交（占据/自由都能写，故增减都会传播）。
  // 只在 fine 窗口按 coarse 对齐滚动/重建时调用，点云与天花板等来源一律不得直写 coarse
  inline void commitCoarseCell(const Eigen::Vector3i &c, bool occupied);
  // 用一个 coarse 平面（沿 axis 的整粗格层）的粗格值，播种对应的 ratio 个 fine 层
  void seedCoarsePlane(int axis, int c_plane);
  inline void posToFineIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &g);
  inline void posToCoarseIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &c);
  inline bool isFineActive(const Eigen::Vector3i &g);
  inline size_t toFineAddress(const Eigen::Vector3i &g);
  inline size_t toCoarseAddress(const Eigen::Vector3i &c);
  // coarse 界外读取：越界不再夹到边界格，而是按「无记忆=自由」返回
  inline char readCoarseCell(const Eigen::Vector3i &c);
  // 多层级查询（fine 窗口优先，其次 coarse），不受全局地图边界限制
  inline int multilevelOccupancy(const Eigen::Vector3d &pos, bool raw_only);

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

  std::atomic<bool> is_map_updating_{false};  // 多层级下写者是 cloud worker，vis 只读

  /* 脱困窗口：window 内忽略膨胀层（只认原始命中格），由 EGOReplanFSM 开关 */
  bool escape_active_{false};
  Eigen::Vector3d escape_center_{Eigen::Vector3d::Zero()};
  double escape_half_{0.5};

  // 机体首次飞出 coarse（全局地图）范围时提示一次，回到范围内复位
  bool warned_outside_coarse_{false};
  // 膨胀盒越出 fine 窗口（配置异常）时提示一次：该情形不写 coarse，只保留窗口内部分
  bool warned_inflate_out_window_{false};

  /* ================= cloud 独立线程 + fine 双缓冲（仅多层级） =================
     建图整帧（roll/clear/mark/ceil/safezone）挪到专用线程；executor 线程只做"能不能处理"
     与必须留在 executor 的事：取 TF（tf2::BufferCore 内部无锁，不能跨线程 lookup）、
     快照位姿、解析/过滤/变换点云，然后把这一帧交给 worker（最新帧优先，排队中的旧帧顶掉）。

     规划器与可视化都不碰"正在被写的"缓冲：
       live = md_.occupancy_buffer_fine_  唯一写者是 cloud worker
       view = fine_view_                 读者唯一入口
     帧末同步只搬运本帧真正翻转过的格子（fine_diff_），代价随场景变化量而非窗口体积增长；
     另按槽位每帧巡检 1/kFineFullSyncEvery，使全量同步仍保持 20 帧一轮而不出现单帧拷贝尖峰；
     整窗拷贝只留给窗口重建与 diff 记录溢出。

     为什么必须双缓冲而不是共享加锁：一帧内是「先清 clear 盒、再打 mark」，读者插进这两步
     之间会在真实障碍处读到 FREE（幻影自由空间）。读到上一帧的值只损失一帧延迟，无害。
     双缓冲把读者能看到的"半帧状态"从整帧（实机 ~150ms）压缩到帧末同步那一小段，
     且同步期间每个格子仍是某个全局位置的合法值。另一个致命点是环形寻址：窗口平移会让旧槽位
     代表别的全局位置，所以视图边界必须与内容配套发布（syncFineView 的三段式收缩-回放-展开）。 */
  struct CloudFrame
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;               // 已解析、已过滤、已在 world 系
    Eigen::Vector3d cam_pos = Eigen::Vector3d::Zero();  // 与该点云同时刻的传感器原点（world）
  };

  std::thread cloud_worker_;
  std::mutex cloud_mtx_;
  std::condition_variable cloud_cv_;
  CloudFrame cloud_pending_;
  bool cloud_has_pending_{false};
  std::atomic<bool> cloud_worker_run_{false};

  /* live 缓冲的写锁：worker 独占整帧；外部偶发写（resetBuffer 区间清除）等锁即可 */
  std::mutex live_write_mtx_;
  /* 跨线程位姿：odomCallback 在 executor 写 odom_pos_，cloudCallback 加锁快照后交给 worker */
  std::mutex cam_mtx_;

  std::vector<char> fine_view_;     // 读侧快照，与 live 等长、同槽位寻址
  std::vector<uint64_t> fine_diff_; // 本帧变更记录：槽位地址 << 2 | 值
  bool fine_full_dirty_{false};     // diff 不可用（整窗重建 / 记录溢出），同步走一次整窗拷贝
  size_t fine_diff_cap_{0};         // 记录数超过它，整窗拷贝更划算

  /* 视图元数据用 seqlock 发布：读者无锁，只在边界改动的瞬间重试 */
  struct FineViewMeta
  {
    Eigen::Vector3i lo, hi;     // fine 有效全局索引区间 [lo, hi)
    Eigen::Vector3i bmin, bmax; // 本帧点云包围盒（fine 全局索引，可视化用）
    Eigen::Vector3d cam;        // 与该视图配套的相机原点
  };
  FineViewMeta view_meta_;
  std::atomic<uint64_t> view_seq_{0};

  /* 全量同步的节奏。不做"每 N 帧一次整窗 memcpy"：实机 res=0.05 / range=16m 时 fine 窗口是
     659x659x339 ≈ 140MB，整窗一次拷贝在 RK 上就是几十毫秒，而且这段时间读侧一直在看混合内容。
     改成每帧顺带把缓冲区按槽位连续切成 N 份巡检一份（140MB/20 ≈ 7MB ≈ 1~2ms），
     同样保证任何 diff 漏记/错位最迟一整套 N 帧被纠正，代价恒定可预算。
     按槽位分片之所以安全：发布边界始终是当帧窗口，槽位 s 的内容与它此刻代表的全局位置同源；
     刚滚动进来、语义换过主人的那一层本来就是本帧新铺的，已被 diff 覆盖。 */
  static const int kFineFullSyncEvery = 20;
  size_t sweep_pos_{0};   // 巡检游标（槽位地址）
  size_t sweep_chunk_{0}; // 每帧巡检的槽位数 = ceil(窗口体积 / kFineFullSyncEvery)

  /* coarse 脏点列表（仅可视化）：写者只有 cloud worker，读者是 vis 回调。
     initMap 一次性 reserve 到上限后永不再扩容（避免搬迁使读者拿到失效指针），
     元素先写、后以 release 发布计数，vis 只遍历到该计数 */
  static const size_t kCoarseDirtyCap = 2000000;
  std::atomic<size_t> coarse_dirty_cnt_{0};
  inline void pushCoarseDirty(const Eigen::Vector3d &pos);

  void startCloudWorker();
  void stopCloudWorker();
  void cloudWorkerLoop();
  void processCloudFrame(const CloudFrame &frame);
  void syncFineView();
  inline void setFineLive(size_t adr, char val);
  inline void setFineLiveG(const Eigen::Vector3i &g, char val);
  /* 只升不降的两个语义：膨胀只改写自由格，命中格取 max */
  inline void setFineLiveIfFree(size_t adr, char val);
  inline void setFineLiveMax(size_t adr, char val);
  inline bool readFineViewCell(const Eigen::Vector3i &g, char &val);
  inline bool readViewMeta(FineViewMeta &m);
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
  // 多层级（cloud）模式：fine 环形窗口随相机无限滚动，查询不受全局地图边界限制。
  // 若在此处用 isInMap 拒绝，越界一律返回 -1，而规划器大量 `if (getInflateOccupancy(pt))`
  // 写法把非零（含 -1）当障碍，机体一到 coarse 范围边界就再也规划不出去
  if (mp_.enable_depth_raycast_ && !isInMap(pos))
    return -1;

  // 脱困窗口内只认原始命中格：临时关掉膨胀，给规划器留出空间
  if (escape_active_ && inEscapeWindow(pos))
    return getRawOccupancy(pos);

  return getInflateOccupancyNoEscape(pos);
}

inline int GridMap::multilevelOccupancy(const Eigen::Vector3d &pos, bool raw_only)
{
  Eigen::Vector3i g;
  posToFineIndex(pos, g);

  char val;
  // 读侧只碰 fine_view_：规划器与 cloud worker 之间没有任何锁，seqlock 保证
  // 边界与该格内容取自同一次发布，看到的必然是某帧同步完成时的一致状态
  if (readFineViewCell(g, val))
    return raw_only ? (val == OCC_RAW ? 1 : 0) : (val > OCC_FREE ? 1 : 0);

  // 窗口之外（或视图正在切换）查 coarse 持久记忆。coarse 只覆盖全局地图范围，范围外从未观测过，
  // 按未观测（自由）返回 —— 机体因此可以飞出 coarse 范围，靠滚动的 fine 窗口导航
  Eigen::Vector3i c;
  posToCoarseIndex(pos, c);
  // coarse 只存原始几何（膨胀只在 fine 层做），故 raw_only 与膨胀查询同义
  return readCoarseCell(c) != 0 ? 1 : 0;
}

inline int GridMap::getInflateOccupancyNoEscape(Eigen::Vector3d pos)
{
  if (!mp_.enable_depth_raycast_)
    return multilevelOccupancy(pos, false);

  if (!isInMap(pos))
    return -1;

  // depth 模式：单层全局栅格
  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_inflate_[toAddress(id)] > OCC_FREE ? 1 : 0;
}

inline int GridMap::getRawOccupancy(Eigen::Vector3d pos)
{
  if (!mp_.enable_depth_raycast_)
    return multilevelOccupancy(pos, true);

  if (!isInMap(pos))
    return -1;

  // depth 模式没有原始层，退回膨胀层，避免脱困窗口放行未知障碍
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return md_.occupancy_buffer_inflate_[toAddress(id)] > OCC_FREE ? 1 : 0;
}

inline bool GridMap::inEscapeWindow(const Eigen::Vector3d &pos) const
{
  return escape_active_ &&
         std::fabs(pos(0) - escape_center_(0)) <= escape_half_ &&
         std::fabs(pos(1) - escape_center_(1)) <= escape_half_ &&
         std::fabs(pos(2) - escape_center_(2)) <= escape_half_;
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

// 全局 fine 索引 -> 环形槽位。必须先按有符号数取模再补 n：
// 旧写法 (size_t)g % (size_t)n 依赖 n 整除 2^64（即 n 为 2 的幂）才对负数成立，
// 而窗口边长按 local_update_range 算出的 n（如 659）不是 2 的幂，
// 机体靠近地图边界使 fine_min_gidx_ 为负时，负索引会落到完全无关的槽位（错位写坏地图）
inline int fine_slot(int g, int n)
{
  int s = g % n;
  return s < 0 ? s + n : s;
}

inline size_t GridMap::toFineAddress(const Eigen::Vector3i &g)
{
  // 绝对取模环形寻址：slot = g mod n，窗口 min 平移时无需搬迁数据
  size_t s0 = (size_t)fine_slot(g(0), md_.fine_n_(0));
  size_t s1 = (size_t)fine_slot(g(1), md_.fine_n_(1));
  size_t s2 = (size_t)fine_slot(g(2), md_.fine_n_(2));
  return (s0 * (size_t)md_.fine_n_(1) + s1) * (size_t)md_.fine_n_(2) + s2;
}

/* ---------------- cloud 线程与 fine 双缓冲 ---------------- */

/* 写 live 缓冲的唯一入口：值真变了才记 diff。mark 的膨胀盒快路径本来就只在
   FREE->INFLATE 时写，所以稳态（静止场景）每帧变更记录很少，同步代价随变化量走 */
inline void GridMap::setFineLive(size_t adr, char val)
{
  char *live = md_.occupancy_buffer_fine_.data();
  if (live[adr] == val)
    return;
  if (!fine_full_dirty_)
  {
    if (fine_diff_.size() >= fine_diff_cap_)
      fine_full_dirty_ = true; // 变化量过大：整窗拷贝更划算
    else
      fine_diff_.push_back((uint64_t(adr) << 2) | uint64_t(val));
  }
  live[adr] = val;
}

inline void GridMap::setFineLiveG(const Eigen::Vector3i &g, char val)
{
  setFineLive(toFineAddress(g), val);
}

/* 膨胀盒内：只把自由格升级为膨胀格（原始命中格不能被覆盖） */
inline void GridMap::setFineLiveIfFree(size_t adr, char val)
{
  if (md_.occupancy_buffer_fine_[adr] == OCC_FREE)
    setFineLive(adr, val);
}

/* 命中格：取 max，低态不会把已标的原始障碍降级 */
inline void GridMap::setFineLiveMax(size_t adr, char val)
{
  if (md_.occupancy_buffer_fine_[adr] < val)
    setFineLive(adr, val);
}

/* coarse 脏点追加：容量已在 initMap 一次 reserve 到位，push_back 不会搬迁内存，
 * 因此 vis 侧即使读到老 size 也只会读到仍然有效的旧元素，不会拿到悬垂指针 */
inline void GridMap::pushCoarseDirty(const Eigen::Vector3d &pos)
{
  if (md_.coarse_dirty_pts_.size() >= kCoarseDirtyCap)
    return;
  md_.coarse_dirty_pts_.push_back(pos);
  coarse_dirty_cnt_.store(md_.coarse_dirty_pts_.size(), std::memory_order_release);
}

/* seqlock 读一个 fine 格：把「边界」和「该槽位的内容」夹在同一次发布之内。
   只验边界是不够的——读者可能在"拿到旧边界"与"读槽位"之间被 worker 的帧末同步插进去，
   窗口一旦平移，那个槽位已经代表另一个全局位置，读到的是毫不相干的值。
   seq 前后两次读取不变，就说明这两步之间没有任何同步发生，结果必然合法。
   计数器每帧只变 4 次（收缩、展开各一对奇偶），正常首次即成功；
   连续读不稳（理论不可能）按"窗口外"处理，宁可退回 coarse 也不碰可能错位的槽位 */
inline bool GridMap::readFineViewCell(const Eigen::Vector3i &g, char &val)
{
  for (int spin = 0; spin < 4; ++spin)
  {
    uint64_t s0 = view_seq_.load(std::memory_order_acquire);
    if (s0 & 1)
      continue;
    Eigen::Vector3i lo = view_meta_.lo;
    Eigen::Vector3i hi = view_meta_.hi;
    bool inside = g(0) >= lo(0) && g(1) >= lo(1) && g(2) >= lo(2) &&
                  g(0) < hi(0) && g(1) < hi(1) && g(2) < hi(2);
    char v = inside ? fine_view_[toFineAddress(g)] : OCC_FREE;
    if (s0 != view_seq_.load(std::memory_order_acquire))
      continue;
    if (!inside)
      return false;
    val = v;
    return true;
  }
  return false;
}

inline bool GridMap::readViewMeta(FineViewMeta &m)
{
  for (int spin = 0; spin < 4; ++spin)
  {
    uint64_t s0 = view_seq_.load(std::memory_order_acquire);
    if (s0 & 1)
      continue;
    FineViewMeta t = view_meta_;
    if (s0 != view_seq_.load(std::memory_order_acquire))
      continue;
    m = t;
    return true;
  }
  return false;
}

inline size_t GridMap::toCoarseAddress(const Eigen::Vector3i &c)
{
  return ((size_t)c(0) * (size_t)md_.coarse_voxel_num_(1) + (size_t)c(1)) * (size_t)md_.coarse_voxel_num_(2) +
         (size_t)c(2);
}

/* 读 coarse 格：0 = 自由，非 0 = 占据。越界按「无记忆 = 自由」返回 */
inline char GridMap::readCoarseCell(const Eigen::Vector3i &c)
{
  if (c(0) < 0 || c(1) < 0 || c(2) < 0 ||
      c(0) >= md_.coarse_voxel_num_(0) || c(1) >= md_.coarse_voxel_num_(1) ||
      c(2) >= md_.coarse_voxel_num_(2))
    return 0;
  return md_.occupancy_buffer_coarse_[toCoarseAddress(c)];
}

/* coarse 唯一写入口：整粗格提交，1 字节/粗格（与 ratio 无关）。
   窗口按 coarse 对齐（fine_n_ 是 ratio 的整数倍、fine_min_gidx_ 是 ratio 的倍数），
   所以任何 coarse 格要么整格在窗口内、要么整格在窗口外；滚出时由调用方读完它全部
   ratio^3 个子格再 OR 成 occupied，因此占据和自由都能如实写入（增减都更新）。
   coarse 自身永不膨胀（膨胀只在 fine 层）。 */
inline void GridMap::commitCoarseCell(const Eigen::Vector3i &c, bool occupied)
{
  if (c(0) < 0 || c(1) < 0 || c(2) < 0 ||
      c(0) >= md_.coarse_voxel_num_(0) || c(1) >= md_.coarse_voxel_num_(1) ||
      c(2) >= md_.coarse_voxel_num_(2))
    return;

  size_t addr = toCoarseAddress(c);
  char old_v = md_.occupancy_buffer_coarse_[addr];
  char new_v = occupied ? (char)OCC_RAW : (char)OCC_FREE;
  if (new_v == old_v)
    return;
  md_.occupancy_buffer_coarse_[addr] = new_v;
  // 脏点只在「空 -> 非空」时追加一次；归零后由发布侧按当前值过滤掉
  if (old_v == 0 && new_v != 0)
    pushCoarseDirty(mp_.map_origin_ +
                    (c.cast<double>() + Eigen::Vector3d(0.5, 0.5, 0.5)) * mp_.coarse_resolution_);
}

/* 用一个 coarse 平面（沿 axis 的第 c_plane 层）播种对应的 ratio 个 fine 层。
   窗口按 coarse 对齐，故 a1/a2 两个方向上的 coarse 索引与子格都能从窗口边界直接算出，
   不需要逐格做 floor 除法。越界粗格按「无记忆=自由」播种。 */
inline void GridMap::seedCoarsePlane(int axis, int c_plane)
{
  const int r = mp_.coarse_ratio_;
  const int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
  const int nc1 = md_.fine_n_(a1) / r, nc2 = md_.fine_n_(a2) / r;
  const int base1 = md_.fine_min_gidx_(a1) / r;
  const int base2 = md_.fine_min_gidx_(a2) / r;

  for (int i1 = 0; i1 < nc1; ++i1)
    for (int i2 = 0; i2 < nc2; ++i2)
    {
      Eigen::Vector3i c;
      c(axis) = c_plane;
      c(a1) = base1 + i1;
      c(a2) = base2 + i2;
      const char val = readCoarseCell(c) != 0 ? (char)OCC_RAW : (char)OCC_FREE;

      Eigen::Vector3i g;
      g(axis) = c_plane * r;
      g(a1) = c(a1) * r;
      g(a2) = c(a2) * r;
      for (int u = 0; u < r; ++u)
        for (int v = 0; v < r; ++v)
          for (int w = 0; w < r; ++w)
          {
            Eigen::Vector3i gg = g;
            gg(axis) += u;
            gg(a1) += v;
            gg(a2) += w;
            setFineLive(toFineAddress(gg), val);
          }
    }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif