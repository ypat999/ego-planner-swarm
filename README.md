# ego-planner-swarm (RKDrone fork)

上游 ego-planner-swarm 的 RKDrone 定制副本，由主仓库
`XTDrone2_ego_planner`（分支 `dev/ego-planner-swarm`）以 gitlink 钉定。
上游原版参考：https://github.com/Zhefan-Xu/ego-planner-swarm

## RKDrone 本地改动

### rkdrone_traj_server：SZD 安全区精准降落（近期重点）

文件：`src/planner/plan_manage/src/rkdrone_traj_server.cpp`

- **降落点锚定 world 原点**：SZD 激活时降落 xy 强制为 `(0, 0)`（起飞点），
  z = 目标z + `z_offset`（默认 -2.0，补偿定位/起落架高度），不受返航目标点坐标
  携带的地图匹配偏差影响。
- **闭环前视制导**（替代旧开环爬行）：
  - 水平段：设定点 = 飞机实际位置 + 朝原点 `min(d, speed·T)` 前伸，前馈速度
    `min(speed, d/T)`——偏移立即满力修正、接近时按 `d/T` 自然减速，不再冲过头；
  - 垂直段：xy 钉死原点（边降边拉回），z 独立限速前视下降；
  - 相位切换与到位判定均使用飞机真实距离。
- **速度参数拆分**：`speed` 仅管水平 xy；`descend_speed` 独立控制 z 下降速度。
- 全部参数可在 `XTDrone2_ego_planner/rkdrone_launch/launch/ego_planner_launch.py`
  配置并支持运行时 `ros2 param set /rkdrone_traj_server ...` 热调。

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

**1. 走廊余量回中（`corridor_margin`，主机制）**

原碰撞代价只对「初值路径已进入膨胀区」的片段生成 `base_point`，路径不碰撞时
`cps_.direction[i]` 为空、碰撞代价为 0，窗内位置完全由初值（起点→目标的多项式或上一次轨迹）
继承，没有任何回中力。叠加竖直方向膨胀后净空仅 0.4 m（2 格），表现为「特别爱贴窗框上沿」。

`updateCorridorConstraints()` 在每个控制点沿 ±x/±y/±z 六向探测膨胀面，记录最后一个自由
采样点与方向；`calcCorridorCost()` 对距离小于 `corridor_margin` 的方向施加二次型排斥。
该代价**不依赖 `base_point`、路径不碰撞时也生效**，通道两侧不足 margin 时梯度在中心抵消，
形成稳定平衡点。约束在每次重规划初始化时计算一次，优化期间冻结。

| 参数（主仓 launch） | 当前值 | 说明 |
|---|---|---|
| `optimization/corridor_margin` | 0.5 | 到膨胀面的期望余量 (m)；0 关闭。需覆盖通道半宽/半高才能回中 |
| `optimization/corridor_weight` | 20.0 | 排斥权重；过大与平滑项冲突会导致抖动（建议 10~40） |

`optimization/dist0` 已恢复默认 0.5：它只负责「初值已碰撞」片段的脱困，通道内的持续余量
由 `corridor_margin` 负责。原 A* 距离最大化惩罚（`setDistPenalty`）已删除——A* 种子只在
初值碰撞时才计算，无法提供持续回中力。

**2. 优化权重配套（控制点加密到 0.2 m 后重标定）**

`calcSmoothnessCost` 用的是**未按 ts 归一化**的四阶差分，代价 ∝ 控制点间距⁷；
`calcFeasibilityCost` 的速度项含 `1/ts²`。因此 `manager/control_points_distance`
从 0.5 降到 0.2 后必须同步重标定：

| 参数 | 当前值 | 原因 |
|---|---|---|
| `optimization/lambda_smooth` | 10.0（原 1.0） | 0.2 m 时间距使平滑代价缩小约两个数量级，需补偿 |
| `optimization/lambda_feasibility` | 0.1（原 0.5） | `ts` 缩小 2.5 倍使速度项等效放大 6.25 倍，需抵消 |
| `optimization/lambda_fitness` | 1.0（原 0.5） | 只作用于 refine 时间重分配阶段；rebound 优化里没有 fitness 项 |

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

定位存在偏差，书面航点无法保证落在真实窗心；居中完全依赖走廊余量代价 + 碰撞代价
（必须经避障通过），航点只负责给出穿窗方向。

### plan_env：横纵向独立膨胀、天花板膨胀带、大地图寻址

机体在规划链路里是**质点**——A*（`checkOccupancy`）、FSM 验收（`checkTrajectoryFeasibility`）、
优化器碰撞代价都只查 `occupancy_buffer_inflate_`，全部安全余量只能由膨胀提供。本轮改动：

- **新增 `grid_map/obstacles_inflation_z`（默认 0.2）**：`cloudCallback` 里 z 向原本硬编码
  `inf_step_z = 1`，既不跟随 `obstacles_inflation`、又随 `resolution` 变动（安全性绑在性能参数上）。
  现在横纵向各自量化：横向对应桨叶扫掠半径，纵向对应机体半高，互不牵连。
- **虚拟天花板禁飞带厚度改为 `max(1, inf_step_z)` 格**（原硬编码 1 格，`obstacles_inflation_z`
  调大时天花板不跟着留余量），并对 `ceil_id` 做了体素索引钳位——`virtual_ceil_height` 落在地图外时，
  `toAddress` 不会越出缓冲区而是串到相邻列的低 z 行，凭空造出大片假障碍。
- **量化的下限保护**：`inf_step`/`inf_step_z` 取 `max(0, ceil(...))`。`obstacles_inflation` 的
  declare 默认值是 −1.0，负数会让整个膨胀圈静默失效；`inf_step_z` 若为 0 会让天花板整层消失。
- **启动自检日志**（配置值 ≠ 生效值，避免"配了却没生效"的反复排查）：

  ```text
  # inflate_z=0.2（declare 默认，@res 0.125 跳档到 2 格）
  膨胀量化: 横向0.25m->2格(0.25m/侧), 纵向0.20m->2格(0.25m/侧), res=0.125m, 体素3200 x 3200 x 160
  # inflate_z=0.125（主仓 launch 现配，整除，纵向省 1 格）
  膨胀量化: 横向0.25m->2格(0.25m/侧), 纵向0.125m->1格(0.125m/侧), res=0.125m, 体素3200 x 3200 x 160
  ```

- **`toAddress` / `buffer_size` 全链 `size_t` 化**：res 0.1 @400×400×20 的体素数 3.2e9 超过
  `INT_MAX`，int 相乘溢出成负数 → `std::length_error` → 节点 abort。边界判断同步由
  `idx < 0 || idx >= num0*num1*num2` 改为 `idx >= occupancy_buffer_inflate_.size()`。
- 点云模式（`enable_depth_raycast: false`）下不再分配 `occupancy_buffer_` 等五块随全图尺寸
  增长的大缓冲，只留 inflate 一张 `char` 表。

> 膨胀的量化公式、不同 `resolution` 下开口能剩几格、以及 0.8 m 窗的选型判据，
> 统一记在主仓 `README.md` 的「障碍膨胀与体素量化」一节。
>
> 死代码说明：`clearAndInflateLocalMap()` + `inflatePoint()` 仍是各向同性立方且未接入
> 任何调用者；活跃路径只有 `cloudCallback`。深度射线路径 `setOccupied()` 写单格、不膨胀
> （`enable_depth_raycast: false` 时不执行）。

### 其他既有本地改动

- `planning/avoid.h`：Fast-DDS XML 静态加载路径按主机名自适应；
- FSMA/bspline 本地策略：同向目标点跳过急停直接重规划等（历史提交）。

## 部署提醒

launch 配置属主仓库；本仓库改动需在上机环境
`colcon build --packages-select plan_env path_searching bspline_opt ego_planner` 后生效。
`plan_env` 的头文件（如 `MapParams` 成员）参与下游编译，只编 `ego_planner` 会用到旧的
`plan_env` 产物，出现"改了没生效"或直接 ABI 不匹配。
