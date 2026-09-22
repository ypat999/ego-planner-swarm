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

### 其他既有本地改动

- `planning/avoid.h`：Fast-DDS XML 静态加载路径按主机名自适应；
- FSMA/bspline 本地策略：同向目标点跳过急停直接重规划等（历史提交）。

## 部署提醒

launch 配置属主仓库；本仓库改动需在上机环境 `colcon build --packages-select ego_planner` 后生效。
