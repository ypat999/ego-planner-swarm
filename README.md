# ego-planner-swarm (XTDrone2 fork)

上游 ego-planner-swarm 的 XTDrone2 定制副本，由主仓库
`XTDrone2_ego_planner`（分支 `dev/ego-planner-swarm`）以 gitlink 钉定。
上游原版参考：https://github.com/Zhefan-Xu/ego-planner-swarm

## XTDrone2 本地改动

### xtd2_traj_server：SZD 安全区精准降落（近期重点）

文件：`src/planner/plan_manage/src/xtd2_traj_server.cpp`

- **降落点锚定 world 原点**：SZD 激活时降落 xy 强制为 `(0, 0)`（起飞点），
  z = 目标z + `z_offset`（默认 -2.0，补偿定位/起落架高度），不受返航目标点坐标
  携带的地图匹配偏差影响。
- **闭环前视制导**（替代旧开环爬行）：
  - 水平段：设定点 = 飞机实际位置 + 朝原点 `min(d, speed·T)` 前伸，前馈速度
    `min(speed, d/T)`——偏移立即满力修正、接近时按 `d/T` 自然减速，不再冲过头；
  - 垂直段：xy 钉死原点（边降边拉回），z 独立限速前视下降；
  - 相位切换与到位判定均使用飞机真实距离。
- **速度参数拆分**：`speed` 仅管水平 xy；`descend_speed` 独立控制 z 下降速度。
- 全部参数可在 `XTDrone2_ego_planner/xtd2_launch/launch/ego_planner_launch.py`
  配置并支持运行时 `ros2 param set /xtd2_traj_server ...` 热调。

| 参数（namespace `safe_zone_descent/`） | 默认 | 说明 |
|---|---|---|
| `enabled` | false(launch: true) | SZD 降落总开关 |
| `speed` | 0.5 | xy 水平修正速度上限 (m/s) |
| `descend_speed` | 0.5(launch: 0.2) | z 下降速度上限 (m/s) |
| `lookahead_time` | 0.5 | 前视时间 T (s)，下限 0.1 |
| `yaw_speed` | 1.0 | 降落 yaw 对准速度 (rad/s) |
| `zone_size_x/y/z` | 1.0 | 安全区尺寸，进入即切 SZD |
| `position_threshold` | 0.05 | 到位判定阈值 (m) |
| `z_offset` | -2.0 | 降落 z 偏移 (m) |

配套：返航阶段的 `map->odom` 厘米级校正由 `lidar_localization_ros2` 的
"原点基准匹配"负责（见其 README），两者共同构成精准降落链路。

### 穿窗居中与狭窄通道自动减速

面向 1.8 m × 1.2 m 窗洞的穿越场景，目标是**宁可绕远也走窗心**，并降低贴边擦碰风险。

**1. A* 种子路径居中（`dyn_a_star`）**

`setDistPenalty(weight, radius)` 在节点周围球面采样 14 点查膨胀占据，离障碍越近边代价越大，
使 A* 种子路径偏向自由空间中部。参数见主仓 launch：

| 参数 | 当前值 | 说明 |
|---|---|---|
| `manager/astar_penalty_weight` | 3.0 | 惩罚强度；0 关闭（零额外开销）；过大导致过度绕路 |
| `manager/astar_penalty_radius` | 1.0 | 采样球半径 (m)，应小于规划视野 |

**2. 优化权重配套（控制点加密到 0.2 m 后重标定）**

`calcSmoothnessCost` 用的是**未按 ts 归一化**的四阶差分，代价 ∝ 控制点间距⁷；
`calcFeasibilityCost` 的速度项含 `1/ts²`。因此 `manager/control_points_distance`
从 0.5 降到 0.2 后必须同步重标定：

| 参数 | 当前值 | 原因 |
|---|---|---|
| `optimization/lambda_smooth` | 10.0（原 1.0） | 0.2 m 时间距使平滑代价缩小约两个数量级，需补偿 |
| `optimization/lambda_feasibility` | 0.1（原 0.5） | `ts` 缩小 2.5 倍使速度项等效放大 6.25 倍，需抵消 |
| `optimization/lambda_fitness` | 0.5 | 米制，不受间距影响；调低让碰撞代价主导居中 |

> 代价：控制点数约 2~2.5 倍，单次重规划耗时约为 0.5 m 时的 2.5~3.5 倍。
> FSM 以 1 s 周期重规划、traj_server 独立 100 Hz 发布，几十至一百余毫秒的规划耗时可接受；
> 实际值看日志 `total time:...,optimize:...,refine:...`。

**3. B-spline 生成后按最小余量自动减速（`BsplineOptimizer::getMinClearance()`）**

- 实现：`reboundReplan` 在 STEP2（rebound 优化）之后、构造 `UniformBspline` 之前，
  遍历 `cps_.base_point` / `cps_.direction` 求沿轨迹到膨胀障碍面的最小有符号距离。
- 控制点是纯几何量，`ts`（结点间隔）决定时间尺度：**放大 `ts` 只降速、不改路径**。
  余量从阈值线性降到 0 时，速度比例从 1.0 降到 `narrow_vel_ratio`。
- 日志：`[narrow] min_clearance=... < ... , 减速到 ...% (ts=...)`。

| 参数（主仓 launch） | 当前值 | 说明 |
|---|---|---|
| `manager/narrow_clearance_threshold` | 1.0 | 自由余量阈值 (m)，低于此值开始减速 |
| `manager/narrow_vel_ratio` | 0.4 | 余量→0 时的速度比例（默认速度的 40%） |

**4. 不使用「窗口中心航点」强制居中**

定位存在偏差，书面航点无法保证落在真实窗心；居中完全依赖 A* 距离惩罚 + 碰撞代价
（必须经避障通过），航点只负责给出穿窗方向。

### 其他既有本地改动

- `planning/avoid.h`：Fast-DDS XML 静态加载路径按主机名自适应；
- FSMA/bspline 本地策略：同向目标点跳过急停直接重规划等（历史提交）。

## 部署提醒

launch 配置属主仓库；本仓库改动需在上机环境 `colcon build --packages-select ego_planner` 后生效。
