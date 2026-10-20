// #include <fstream>
#include <ego_planner/planner_manager.h>
#include <thread>
#include <cmath>
#include <algorithm>
#include "visualization_msgs/msg/marker.hpp" // zx-todo

namespace ego_planner
{

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis)
  {
    node_ = node;
    node->declare_parameter("manager/max_vel", -1.0);
    node->declare_parameter("manager/max_acc", -1.0);
    node->declare_parameter("manager/max_jerk", -1.0);
    node->declare_parameter("manager/feasibility_tolerance", 0.0);
    node->declare_parameter("manager/control_points_distance", -1.0);
    node->declare_parameter("manager/planning_horizon", 5.0);
    node->declare_parameter("manager/use_distinctive_trajs", false);
    node->declare_parameter("manager/drone_id", -1);
    node->declare_parameter("manager/narrow_clearance_threshold", 0.6);
    node->declare_parameter("manager/narrow_vel_ratio", 0.4);

    node->get_parameter("manager/max_vel", pp_.max_vel_);
    node->get_parameter("manager/max_acc", pp_.max_acc_);
    node->get_parameter("manager/max_jerk", pp_.max_jerk_);
    node->get_parameter("manager/feasibility_tolerance", pp_.feasibility_tolerance_);
    node->get_parameter("manager/control_points_distance", pp_.ctrl_pt_dist);
    node->get_parameter("manager/planning_horizon", pp_.planning_horizen_);
    node->get_parameter("manager/use_distinctive_trajs", pp_.use_distinctive_trajs);
    node->get_parameter("manager/drone_id", pp_.drone_id);
    // Validate critical physical limits to avoid invalid planning results
    if (pp_.max_vel_ <= 0.0)
    {
      RCLCPP_WARN(node->get_logger(), "Invalid manager/max_vel (<=0): %f. Setting to default 2.0 m/s.", pp_.max_vel_);
      pp_.max_vel_ = 2.0;
    }
    if (pp_.max_acc_ <= 0.0)
    {
      RCLCPP_WARN(node->get_logger(), "Invalid manager/max_acc (<=0): %f. Setting to default 3.0 m/s^2.", pp_.max_acc_);
      pp_.max_acc_ = 3.0;
    }
    if (pp_.ctrl_pt_dist <= 0.0)
    {
      RCLCPP_WARN(node->get_logger(), "Invalid manager/control_points_distance (<=0): %f. Setting to default 0.5 m.", pp_.ctrl_pt_dist);
      pp_.ctrl_pt_dist = 0.5;
    }
    
    // 验证无人机ID
    if (pp_.drone_id < 0)
    {
      RCLCPP_WARN(node->get_logger(), "无人机ID为负值(%d)，设置为单机模式(ID=0)", pp_.drone_id);
      pp_.drone_id = 0;
    }

    // 输出重要参数配置
    RCLCPP_INFO(node->get_logger(), "=== EGO Planner 参数配置 ===");
    RCLCPP_INFO(node->get_logger(), "最大速度: %.2f m/s (ego 默认速度，非外部输入上限)", pp_.max_vel_);
    RCLCPP_INFO(node->get_logger(), "最大加速度: %.2f m/s^2", pp_.max_acc_);
    RCLCPP_INFO(node->get_logger(), "最大加加速度: %.2f m/s^3", pp_.max_jerk_);
    RCLCPP_INFO(node->get_logger(), "控制点距离: %.2f m", pp_.ctrl_pt_dist);
    RCLCPP_INFO(node->get_logger(), "规划时域: %.2f s", pp_.planning_horizen_);
    RCLCPP_INFO(node->get_logger(), "可行性容差: %.3f", pp_.feasibility_tolerance_);
    RCLCPP_INFO(node->get_logger(), "使用独特轨迹: %s", pp_.use_distinctive_trajs ? "是" : "否");
    RCLCPP_INFO(node->get_logger(), "无人机ID: %d", pp_.drone_id);
    RCLCPP_INFO(node->get_logger(), "================================");

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    // grid_map_->initMap(nh);
    grid_map_->initMap(node);

    bspline_optimizer_.reset(new BsplineOptimizer);
    // bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setParam(node);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    // 狭窄通道自动减速参数
    node->get_parameter("manager/narrow_clearance_threshold", narrow_clearance_threshold_);
    node->get_parameter("manager/narrow_vel_ratio", narrow_vel_ratio_);
    RCLCPP_INFO(node->get_logger(), "狭窄通道减速: threshold=%.2fm, vel_ratio=%.2f", narrow_clearance_threshold_, narrow_vel_ratio_);

    visualization_ = vis;
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    static int count = 0;
    // printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      // cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);
    bspline_optimizer_->setLocalTargetVel(local_target_vel);

    rclcpp::Time t_start = rclcpp::Clock().now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    /*** STEP 1: INIT
    根据起始点和目标点的距离计算首个时间步长ts,向量的模大于0.1则用1.5倍否则用5倍
    ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      // 这里如果正常进入if（通常为初次生成），则do部分只进行一次，即只清空一次点集；若进入else则有可能对异常情况重置flag_regenerate并再do一次
      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;
        // 用于存储生成的轨迹
        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        // 用全速（而非仅目标方向投影）计算减速时间/距离
        // 多项式匹配完整 start_vel 矢量，横向速度也需要时间阻尼
        double v0 = start_vel.norm();
        if (v0 < 0.1) v0 = 0.0;  // 极低速视为静止

        double time;
        double d_decel = v0 * v0 / (2.0 * pp_.max_acc_);  // 减速距离
        double t_decel = v0 / pp_.max_acc_;                // 减速时间
        double d_remain = dist - d_decel;
        if (d_remain <= 0.0)
        {
          time = t_decel;
        }
        else if (pow(pp_.max_vel_, 2) / pp_.max_acc_ > d_remain)
        {
          time = t_decel + sqrt(d_remain / pp_.max_acc_);
        }
        else
        {
          time = t_decel + 2.0 * pp_.max_vel_ / pp_.max_acc_
               + (d_remain - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_;
        }

        if (!flag_randomPolyTraj)
        // false生成一段单一的多项式轨迹，true生成一个包含随机插入点的轨迹
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (rclcpp::Clock().now() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // 将轨迹变为B样条轨迹
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    if (getenv("EGO_DIAG"))
    {
      double L = 0, seg_max = 0, seg_min = 1e9;
      for (size_t i = 1; i < point_set.size(); ++i)
      {
        double d = (point_set[i] - point_set[i - 1]).norm();
        L += d;
        seg_max = std::max(seg_max, d);
        seg_min = std::min(seg_min, d);
      }
      RCLCPP_WARN(node_->get_logger(),
                  "[diag-seed] polyInit=%d seed_pts=%zu d_to_goal=%.2fm ts=%.3f ctrl=%d seedLen=%.2fm seg[min/max]=%.3f/%.3f",
                  (int)flag_polyInit, point_set.size(), (start_pt - local_target_pt).norm(), ts,
                  (int)ctrl_pts.cols(), L, seg_min, seg_max);
    }

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);
    // 计算时间差并更新时间
    auto now = rclcpp::Clock().now();
    t_init = now - t_start;
    t_start = now;

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = rclcpp::Clock().now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2);
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = rclcpp::Clock().now() - t_start;
      // static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    // cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = rclcpp::Clock().now();

#if 0 // 临时关闭窄路减速（排查复杂避障打结用）：恢复时删除本行与块尾的 #endif
    // ==== 狭窄通道自动减速（局部）：只在实际余量不足的路段放慢，其余路段保持原速 ====
    // B 样条几何已定，这里只改时间参数化：沿轨迹把现有速度乘以
    // r(s) = clamp(clearance/threshold, narrow_vel_ratio, 1)，
    // 再按虚拟时间 T = ∫dt/r(s) 等间隔重采样控制点。
    // 窄处控制点变密（同路径走得更慢），宽处保持原密度（维持原速度和原时间分配），
    // 因此起停段的加减速、目标点减速都不受影响。
    {
      const double th = narrow_clearance_threshold_;
      const double dt_samp = 0.05;  // 沿轨迹采样时间步长
      UniformBspline raw(ctrl_pts, 3, ts);
      const double dur = raw.getTimeSum();

      if (dur > 1e-3 && ts > 1e-3)
      {
        const int n = std::max(2, (int)std::ceil(dur / dt_samp) + 1);
        std::vector<Eigen::Vector3d> pts(n);
        std::vector<double> tsamp(n, 0.0), arc(n, 0.0), ratio(n, 1.0);

        double min_c = th;
        for (int i = 0; i < n; ++i)
        {
          tsamp[i] = std::min(dur, i * dt_samp);
          pts[i] = raw.evaluateDeBoorT(tsamp[i]);
          if (i > 0)
            arc[i] = arc[i - 1] + (pts[i] - pts[i - 1]).norm();

          const double c = bspline_optimizer_->getClearanceAt(pts[i], th);
          // c == 0 表示该采样点本身就在膨胀区里：贴地起降（地面膨胀）或轨迹穿障。
          // 这属于"不在可通行空间"，不是"通道窄"，不参与减速判定；
          // 按点跳过而不是把整条轨迹判窄，避免贴地段把整条航线压到 40% 速度。
          if (c <= 1e-3)
            continue;
          min_c = std::min(min_c, c);
          ratio[i] = std::max(narrow_vel_ratio_, std::min(1.0, c / th));
        }

        if (min_c < th)
        {
          // 窄口前后各扩展 0.5m 进入减速区，再做 0.25m 滑动平均，避免速度阶跃
          std::vector<double> r_min = ratio;
          for (int i = 0; i < n; ++i)
          {
            for (int j = i; j >= 0 && arc[i] - arc[j] <= 0.5; --j)
              r_min[i] = std::min(r_min[i], ratio[j]);
            for (int j = i; j < n && arc[j] - arc[i] <= 0.5; ++j)
              r_min[i] = std::min(r_min[i], ratio[j]);
          }

          std::vector<double> r_smooth(n, 1.0);
          for (int i = 0; i < n; ++i)
          {
            double sum = 0.0;
            int cnt = 0;
            for (int j = i; j >= 0 && arc[i] - arc[j] <= 0.25; --j)
            {
              sum += r_min[j];
              ++cnt;
            }
            for (int j = i + 1; j < n && arc[j] - arc[i] <= 0.25; ++j)
            {
              sum += r_min[j];
              ++cnt;
            }
            r_smooth[i] = cnt > 0 ? sum / cnt : 1.0;
          }

          // 虚拟时间：宽处每步 dt，窄处拉伸为 dt/r
          std::vector<double> tv(n, 0.0);
          double slow_len = 0.0;
          for (int i = 1; i < n; ++i)
          {
            tv[i] = tv[i - 1] + (tsamp[i] - tsamp[i - 1]) / std::max(0.05, r_smooth[i]);
            if (r_smooth[i] < 0.999)
              slow_len += arc[i] - arc[i - 1];
          }

          const double t_total = tv.back();
          if (t_total > dur * 1.001)
          {
            const int K = std::max(6, (int)std::lround(t_total / ts));
            const double dt_new = t_total / K;

            std::vector<Eigen::Vector3d> point_set;
            point_set.reserve(K + 1);
            int idx = 0;
            for (int k = 0; k <= K; ++k)
            {
              const double tk = std::min(t_total, k * dt_new);
              while (idx < n - 2 && tv[idx + 1] < tk)
                ++idx;
              const double w = (tk - tv[idx]) / std::max(1e-6, tv[idx + 1] - tv[idx]);
              point_set.push_back((1.0 - w) * pts[idx] + w * pts[idx + 1]);
            }
            point_set.back() = pts.back();  // 终点与原轨迹对齐

            Eigen::MatrixXd ctrl_pts_new;
            UniformBspline::parameterizeToBspline(dt_new, point_set, start_end_derivatives, ctrl_pts_new);
            if (ctrl_pts_new.cols() >= 4)
            {
              ctrl_pts = ctrl_pts_new;
              ts = dt_new;
            }

            RCLCPP_INFO(node_->get_logger(),
                        "[narrow] min_clearance=%.2fm < %.2fm, 局部减速段 %.1f/%.1fm (最低 %.0f%%, ts=%.3fs)",
                        min_c, th, slow_len, arc.back(), narrow_vel_ratio_ * 100.0, ts);
          }
        }
      }
    }
#endif

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    double refine_ratio = 1.0;
    if (pp_.drone_id <= 0)
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        refine_ratio = ratio;
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        RCLCPP_ERROR(rclcpp::get_logger("ego_planner"), "IN SWARM MODE, REFINE DISABLED!");
      }
    }

    // t_refine = ros::Time::now() - t_start;
    t_refine = rclcpp::Clock().now() - t_start;

    // 诊断：输出本条轨迹的时长/长度/平均与峰值速度，用于判断“整体偏慢”来自哪里
    // （polyInit=0 表示走了旧轨迹采样种子；refine_ratio>1 表示 STEP3 被整体拉长时间）
    if (getenv("EGO_DIAG"))
    {
      const double dur = pos.getTimeSum();
      double len = 0.0, vmax = 0.0;
      const int N = 40;
      Eigen::Vector3d prev = pos.evaluateDeBoorT(0.0);
      for (int i = 1; i <= N; ++i)
      {
        const Eigen::Vector3d p = pos.evaluateDeBoorT(dur * i / N);
        len += (p - prev).norm();
        vmax = std::max(vmax, (p - prev).norm() * N / std::max(1e-6, dur));
        prev = p;
      }
      RCLCPP_INFO(node_->get_logger(),
                  "[diag-speed] polyInit=%d ts=%.3f refine_ratio=%.2f ctrl=%d dur=%.2fs len=%.2fm vavg=%.2f vmax=%.2f d_target=%.2fm",
                  (int)flag_polyInit, ts, refine_ratio, (int)pos.getControlPoint().cols(), dur, len,
                  len / std::max(1e-3, dur), vmax, (start_pt - local_target_pt).norm());
    }

    // 末段末端对齐：B 样条是用采样点做加权最小二乘拟合出来的，末端并不精确等于局部目标
    // （种子采样只取 t < time，且速度/加速度行权重远高于位置行）。局部目标只是 7.5m 视野外的
    // 中间点时这点残差无害，但局部目标就是最终目标时，残差会让飞机停在目标点之前，
    // 而 FSM 的到达判定只看时间、traj_server 的吸附又只有 0.5m。
    // 这里把最后 3 个控制点整体平移 end_err：B 样条末端 = (Q_{n-3}+4Q_{n-2}+Q_{n-1})/6，
    // 三点同平移即末端精确平移 end_err，且相邻差值不变（末端速度/加速度保持）。
    if (flag_final_target_)
    {
      const double dur = pos.getTimeSum();
      Eigen::Vector3d end_err = local_target_pt - pos.evaluateDeBoorT(dur);
      Eigen::MatrixXd cps = pos.getControlPoint();
      const double ts_now = dur / std::max(1, (int)cps.cols() - 3);
      for (int i = cps.cols() - 3; i < cps.cols(); ++i)
        cps.col(i) += end_err;
      pos = UniformBspline(cps, 3, ts_now);
      if (end_err.norm() > 0.05)
        RCLCPP_INFO(node_->get_logger(), "[final] 末端对齐 %.2fm 到目标点", end_err.norm());
    }

    // save planned results
    // 用规划开始时刻作为 start_time，让 traj_server 计算 t_cur 时自然包含优化延迟，
    // 轨迹位置与飞机实际位置对齐，避免因时间差导致拉回
    updateTrajInfo(pos, t_start);

    static double sum_time = 0;
    static int count_success = 0;

    sum_time += (t_init + t_opt + t_refine).seconds();

    count_success++;

    // cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), rclcpp::Clock().now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    // if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
    if (local_data_.start_time_.seconds() < 1e9)
      return false;

    // double my_traj_start_time = local_data_.start_time_.toSec();
    // double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();
    double my_traj_start_time = local_data_.start_time_.seconds();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.seconds();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = rclcpp::Clock().now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    /*挨个读取点并计算点距判断是否需要插点，随后计算插点并写入矩阵，最后根据插点数量生成全局轨迹
      最终返回值为是否规划成功的布尔值 */
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = rclcpp::Clock().now();

    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }

    // lengthenTime 只拉长节点、不改变几何：同一条曲线的速度变为 1/ratio、加速度变为 1/ratio^2。
    // parameterizeToBspline 解的是最小二乘，速度行权重 ~1/(2dt)、加速度行权重 ~1/dt^2，
    // 远大于位置行权重 1/6。若边界导数仍用缩放前的值，巨大的边界残差会被摊到整条控制点链上，
    // 表现为空旷处出现与障碍无关、从起点开始逐渐衰减的周期性波纹。
    vector<Eigen::Vector3d> deriv = start_end_derivative;
    if (deriv.size() == 4 && std::abs(ratio - 1.0) > 1e-6)
    {
      const double r2 = ratio * ratio;
      deriv[0] /= ratio;   // 起点速度
      deriv[1] /= ratio;   // 终点速度
      deriv[2] /= r2;      // 起点加速度
      deriv[3] /= r2;      // 终点加速度
    }
    UniformBspline::parameterizeToBspline(dt, point_set, deriv, ctrl_pts);
  }

  double EGOPlannerManager::setMaxVel(double max_vel)
  {
    double v = max_vel;
    // manager/max_vel 只是 ego 自身默认速度，不构成对外部输入的上限：
    // 外部请求（/goal_vel_max、ros2 param set）直接采纳，仅做合法性防御。
    if (!std::isfinite(v) || v <= 0.0)
    {
      RCLCPP_WARN(rclcpp::get_logger("EGOPlannerManager"),
                  "Invalid requested max_vel (%f). Fallback to default 2.0 m/s.", v);
      v = 2.0;
    }

    pp_.max_vel_ = v;
    // 同步优化器限值，避免“时间按新速度排、优化器仍按旧速度”的不一致
    bspline_optimizer_->setDynamicLimits(v, pp_.max_acc_);

    // 回写 optimization/max_vel，使参数读出值与实际生效值一致（该参数无 on-set 回调，不会递归）
    if (node_)
    {
      node_->set_parameter(rclcpp::Parameter("optimization/max_vel", v));
    }

    return v;
  }

} // namespace ego_planner
