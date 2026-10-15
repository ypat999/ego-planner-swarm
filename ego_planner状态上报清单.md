# ego_planner 状态上报清单（planner ↔ task_dispatcher 互通）

> 版本：v1.1（2026-10-14）  
> v1.1 变更：**撤销 `/mission_status` 新话题**——心跳与 FSM 状态一律走既有的 `/mission_event`（`event:"HEARTBEAT"`），planner 侧对外仍只占这一个话题。  
> 取证范围：`xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/**` + `uav_service/src/task_dispatcher/**`  
> 协议口径：对齐仓库根 `系统失效降级方案设计.md` §14「任务面接口契约」——**String+JSON，不新增自定义 msg、不新增话题、不新增事件名**（新事件一律复用 §14 已列出的 `GOAL_UNREACHABLE / GOAL_REJECTED / SAFETY_HOLD / ARRIVED / NOT_READY / RETURN_*`）。  
> 用途：本文是 planner 侧「已上报 / 应上报」的唯一对账清单，供后续 ego 与编排侧同步时逐项勾选。

---

## 1. 链路事实（先对齐通路，再看上报）

```
task_dispatcher/perform_action_manager ──/goal_pose_3d──▶ xtd2_communication(offboard 闸门)
                                                        └─闸门放行后─▶ /move_base_simple/goal
                                                                         ├─▶ ego_planner_node (EGOReplanFSM)  ← 规划与报错归属方
                                                                         └─▶ xtd2_traj_server (下发 position_cmd)

ego_planner_node ──┐
task_dispatcher ───┼──▶ /mission_event (String+JSON，唯一状态出口：失效事件 + 1Hz 心跳合流)
                    └──▶ cruise_uav_task_manager（同话题三源共用，靠 source 判别）
```

| 事实 | 证据 | 对上报的影响 |
|---|---|---|
| planner 报错的**唯一归属节点**是 `ego_planner_node`（FSM），不是 traj_server | [ego_planner_node.cpp#L12-L18](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_planner_node.cpp#L12-L18) | 所有 `source=planner` 的消息都从该节点发 |
| `/mission_event` 已是**多源共用**话题：planner、perform_action、cruise 三处都在发 | [ego_replan_fsm.cpp#L128](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L128)、[perform_action_manager.py#L51](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/perform_action_manager.py#L51)、[cruise_uav_task_manager.py#L140](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/cruise_uav_task_manager.py#L140) | 合流心跳**必须带 `source`** 且消费方先按 source 过滤，否则判活会失真（见 §7 坑 8） |
| 目标点实际订阅 `/move_base_simple/goal`，`/goal_pose_3d` 直订已注释 | [xtd2_traj_server.cpp#L867-L871](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/xtd2_traj_server.cpp#L867-L871)；[ego_replan_fsm.cpp#L141-L147](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L141-L147) | 同函数 [L894](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/xtd2_traj_server.cpp#L894) 日志仍写 `/goal_pose_3d`，排查时勿被误导 |
| goal 上游有 comms 闸门，未就绪时**静默丢弃** | [multirotor_communication.py#L277-L279](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_communication/xtd2_communication/multirotor_communication.py#L277-L279) | 「planner 没反应」有两种成因：goal 没到 / planner 收到但失败。**只有心跳里的 `state` 能把两者区分开** |
| 回调频率：FSM 主循环 100Hz，碰撞检测 20Hz | [ego_replan_fsm.cpp#L72-L76](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L72-L76) | 任何新增上报**必须边沿去重**，否则 100 条/秒刷屏 |
| 节点为单线程 `rclcpp::spin`，且 `planNextWaypoint` 内存在 `while + spin_some` 等待 | [ego_replan_fsm.cpp#L427-L431](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L427-L431) | 上报路径上不得引入阻塞/等待订阅方的逻辑 |

---

## 2. 现状：ego 已经在发/收哪些消息

### 2.1 planner 侧全部发布话题

| 话题 | 类型 | 频率 | 语义 | 能否直接被编排侧消费 |
|---|---|---|---|---|
| `/mission_event` | `std_msgs/String`(JSON) | 事件驱动 | **唯一的失效上报出口**（本文改为：失效事件 + 心跳合一） | ✅ 已被 cruise 订阅 |
| `planning/bspline` | `traj_utils/Bspline` | 每次规划成功 | 轨迹下发给 traj_server | ➖ 非状态语义 |
| `planning/data_display` | `traj_utils/DataDisp` | 100Hz | 5 个裸 `float64 a~e` | ❌ 无字段名，[DataDisp.msg](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/traj_utils/msg/DataDisp.msg) 不可读，禁止当状态口用 |
| `planning/broadcast_bspline_from_planner`、`/drone_*_planning/swarm_trajs` | Bspline / MultiBsplines | 每次规划 | 多机级联 | ➖ 单机不用 |
| `grid_map/occupancy`、`grid_map/occupancy_inflate` | PointCloud2 | ~9Hz / ~2Hz | 障碍点云（原始/膨胀） | ➖ 可视化与取证用，[grid_map.cpp#L236-L237](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_env/src/grid_map.cpp#L236-L237) |
| `goal_point` / `global_list` / `init_list` / `optimal_list` / `a_star_list` | Marker | 事件 | RViz 可视化 | ❌ |
| `/xtdrone2<ns>cmd_trajectory_flu` | `quadrotor_msgs/PositionCommand` | 20Hz | 轨迹设定点（traj_server） | ➖ |
| `/debug_info`、`droneXtoY_pose_err` | String / PoseStamped | 事件 | 多机位姿偏差（drone_detect），[drone_detector.cpp#L25-L41](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/drone_detect/src/drone_detector.cpp#L25-L41) | ➖ 多机才需要 |

### 2.2 planner 侧全部订阅话题

`odom_world`（里程计）、`/move_base_simple/goal`（目标点）、`/traj_start_trigger`（预设航线触发）、`/stop_planning`（编排侧收回控制权）、`grid_map/{cloud,odom,pose,depth}`、`/vins_estimator/extrinsic`、`planning/broadcast_bspline_to_planner`。

> **注意**：`/stop_planning` 是**单向无回执**的。编排侧 [`_publish_stop_planning`](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/cruise_uav_task_manager.py#L506-L512) 发完即假定生效，planner 侧 [L931-L944](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L931-L944) 只打 WARN。→ 由 §4.2 心跳帧里的 `state:"WAIT_TARGET"` 充当隐式回执（G13）。

### 2.3 `/mission_event` 现有命中点（planner 侧全量，仅 2 处）

| 触发条件 | 位置 | payload |
|---|---|---|
| 新下发的航点落在膨胀障碍内 → 拒绝接受 | [ego_replan_fsm.cpp#L377-L387](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L377-L387) | `GOAL_UNREACHABLE{source:planner,reason:OBSTACLE,goal_seq:"NA",flight_epoch:"NA"}` |
| 飞行中最终目标点进入膨胀区 → `EMERGENCY_STOP` | [L1367-L1383](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1367-L1383) | 同上（`last_goal_obstacle_pub_` 边沿去重，[ego_replan_fsm.h#L89](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/include/ego_planner/ego_replan_fsm.h#L89)） |

底层封装 `publishMissionEvent()`：[L1043-L1071](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1043-L1071)（自动补 `event/source/ts`，`json_fields` 为手工拼的键值串，**无状态快照字段**）。

编排侧消费：[`_mission_event_callback`](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/cruise_uav_task_manager.py#L172-L190) 只认 `event==GOAL_UNREACHABLE && source==planner && reason==OBSTACLE`，3s 窗去重后累加 `same_point_retry` → 进 [跳点/返航/迫降状态机](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/cruise_uav_task_manager.py#L204-L261)。

**结论：planner 对外只有一种声音——「目标点在障碍里」，而且不带任何状态快照。** 其余所有失败（无路、机体越界、感知丢失、规划不收敛、目标被拒、FSM 现在处于哪个状态）全部只落 `RCLCPP_*` 日志，编排侧只能靠 5s 停滞 watchdog + 盯位置反推。

---

## 3. 缺口清单：还有哪些信息值得塞进 `/mission_event`

优先级：**P0 = 影响安全决策、必须做**；P1 = 显著减少空等/误判；P2 = 多机或运维观测。

### 3.1 你已点到的三类

| # | 信号 | 判定点（代码已存在，只是没上报） | 现状 | 建议 |
|---|---|---|---|---|
| G1 | **路线有障碍 / 规划不出轨迹** | A\* 段失败 [bspline_optimizer.cpp#L588-L597](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/bspline_opt/src/bspline_optimizer.cpp#L588-L597)；轨迹末端在障碍 [L1385-L1392](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/bspline_opt/src/bspline_optimizer.cpp#L1385-L1392)；rebound 超限 [L1520](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/bspline_opt/src/bspline_optimizer.cpp#L1520) → 层层 `return false` 到 FSM：[L1131-L1142](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1131-L1142)（首发失败）、[L1155-L1165](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1155-L1165)（新目标失败）、[L1171-L1179](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1171-L1179)（重规划失败） | 仅日志；GEN_NEW_TRAJ 以 **100Hz** 空转重试，外部完全看不出「已经放弃了」 | `GOAL_UNREACHABLE{reason:NO_PATH}`，`detail` 带 `stage=first/gen/replan trials=N` |
| G2 | **飞机自己在膨胀区里 / 出图** | 需要新增判定：`grid_map_->getInflateOccupancy(odom_pos_)`。现在全仓对 `getInflateOccupancy` 的调用**只传 `next_wp`/`end_pt_`/轨迹点，从未传机体位置** | 完全没有检测，更没有上报。飞机被障碍包住时 FSM 仍正常跑、仍反复出规划 | `SAFETY_HOLD{reason:VEHICLE_IN_OBS}`；越界单列 `reason:OUT_OF_MAP`（见 §7 坑 1） |
| G3 | **目标点被遮挡** | 现有判定只覆盖「已知有障碍」(`getInflateOccupancy==1`)。**未观测（unknown）区域被当成自由区直接规划**——这恰恰是「遮挡」的主语义。可用 [grid_map.h#L197-L198](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_env/include/plan_env/grid_map.h#L197-L198) 的 `isUnknown()` | 静默按「可达」处理，冲进去才发现 | `GOAL_UNREACHABLE{reason:UNOBSERVED}`（目标未被观测，≠ 有障碍，处置应不同：等一圈感知而不是跳点） |

### 3.2 其他同样只活在日志里的信号（建议一并纳入）

| # | 信号 | 位置 | 现状 | 建议 | 编排侧价值 |
|---|---|---|---|---|---|
| G4 | 深度/点云丢失（感知超时） | [ego_replan_fsm.cpp#L1349-L1355](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1349-L1355)，标志位由 [grid_map.cpp#L837-L842](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_env/src/grid_map.cpp#L837-L842) 置位 | 直接 `EMERGENCY_STOP`，外部只看见飞机停住 | `SAFETY_HOLD{reason:DEPTH_LOST}` | 地图不再可信 → 立即返航，不能继续跳点 |
| G5 | 飞行中轨迹撞障急停 | [L1411-L1434](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1411-L1434)（`Suddenly discovered obstacles. emergency stop!`） | WARN，无 topic | `SAFETY_HOLD{reason:TRAJ_COLLISION}` | 「差点撞上」是返航/迫降阈值的直接输入，现在完全丢失 |
| G6 | odom 数据非法（非数值/过大/四元数不规范/协方差异常） | [L727-L793](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L727-L793) | ERROR 日志（每条坏帧都发，高频） | `SAFETY_HOLD{reason:ODOM_INVALID}`（边沿去重）+ 心跳 `lio:false` | LIO 质量劣化的机内早期证据（§14 的 `LIO_RED` 语义落点） |
| G7 | 目标点被拒（数据非法 / `z < -0.1` / TF 变换失败） | [L511-L543](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L511-L543)、[L567-L572](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L567-L572) | 直接 `return`，**航点被无声吞掉**，编排侧一直等到停滞超时 | `GOAL_REJECTED{reason:INVALID_DATA/Z_INVALID/TF_FAIL, detail:"target=x,y,z"}` | 参数错误 vs 环境不可达必须分开：前者**不该重试** |
| G8 | 目标点被判为重复而被忽略（1cm 内且 <10s） | [L588-L597](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L588-L597) | DEBUG 日志 | `GOAL_REJECTED{reason:DUPLICATE}` | 解释「我发了 goal 但 planner 什么都没做」 |
| G9 | 全局轨迹（前端）生成失败 / duration 非法 | [L437-L440](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L437-L440)、[L398-L407](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L398-L407) | ERROR 日志 | `GOAL_UNREACHABLE{reason:GLOBAL_TRJ_FAIL}` | 与 G1 区分：前端就连不上，重试无意义 |
| G10 | 「目标点在障碍里」被 planner 静默放弃（与已有事件同语义却不上报的两处） | [L1275-L1282](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1275-L1282)（`planFromGlobalTraj` 放弃）、[L1244-L1252](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1244-L1252)（急停恢复时发现目标仍在障碍 → 回 `WAIT_TARGET`） | **同一条判据，只有另外两处会发事件**，这两处不发 → 编排侧漏计数，飞机原地悬停到电量耗尽 | 补齐 `GOAL_UNREACHABLE{reason:OBSTACLE}` | 这是当前**最容易造成静默失声**的一处 |
| G11 | **FSM 状态机全序**（`INIT/WAIT_TARGET/GEN_NEW_TRAJ/REPLAN_TRAJ/EXEC_TRAJ/EMERGENCY_STOP/SEQUENTIAL_START`） | [L1037](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1037)、[L1080](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1080) 只 `cout` | 外部只能「盯位置猜状态」 | `HEARTBEAT.state`（§4.2），状态跃迁时**立即多发一帧** | 你要的「把 fsm_wait 之类传出来」即此项；也是 §14 H8「comms 死=失声」的正解 |
| G12 | 到达确认（轨迹执行完毕） | [L1202-L1217](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1202-L1217) | 无 | `ARRIVED{detail:"target=...,dist=..."} `；心跳常驻 `dist_goal` | 编排侧现在是纯距离判到达（[perform_action_manager.py#L316](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/perform_action_manager.py#L316)），可换成事件/心跳驱动（§14 P-D） |
| G13 | `/stop_planning` 已生效 | [L931-L944](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L931-L944) | 无回执 | 收到 `/stop_planning` 后立即发一帧 `HEARTBEAT{state:"WAIT_TARGET"}`（不新增事件名） | 「降落/返航前必须先把控制权交回飞控」的闭环确认 |
| G14 | SZD 安全区降落已接管 | [xtd2_traj_server.cpp#L284-L291](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/xtd2_traj_server.cpp#L284-L291) | 无 | 心跳 `szd:true`（traj_server 直接在同一话题发 `HEARTBEAT{source:"traj_server"}`，或由 planner 合并） | 编排侧 `_wait_landed` 现在只看 arming 状态，分不清「在降落」还是「在悬停」 |
| G15 | 多机邻居异常（轨迹时间差 >0.25s、traj 数量/阶数不匹配） | [L857-L863](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L857-L863)、[L954-L968](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L954-L968) | ERROR 日志 | P2：`SAFETY_HOLD{reason:NEIGH_STALE, detail:"drone_id=N"}` | 单机编队不用 |
| G16 | 性能退化（融合耗时、规划耗时） | [grid_map.cpp#L247-L249](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_env/src/grid_map.cpp#L247-L249) `fuse_time_/max_fuse_time_` | 无 | P2：心跳 `fuse_ms` | 算力饱和的早期预警（会先表现为「不推进」，再变成 G1） |

---

## 4. 建议协议：`/mission_event` 单话题承载全部

### 4.1 事件与 reason 全集

| event | source | reason | 由本文哪条产生 | 频率 | 语义 |
|---|---|---|---|---|---|
| `HEARTBEAT` | `planner` | `OK` 或最近一次主因 | **G11/G13/G14/G16** | 1Hz + 状态跃迁即发 | 状态快照，兼作存活判定（**不参与任何计数**） |
| `GOAL_UNREACHABLE` | `planner` | `OBSTACLE` | 已有 + **G10** | 边沿 | 目标点落在膨胀障碍内 |
| | | `UNOBSERVED` | G3 | 边沿 | 目标点未被观测（遮挡），非确定障碍 |
| | | `OUT_OF_MAP` | G2/坑1 | 边沿 | 目标点越出建图边界 |
| | | `NO_PATH` | G1 | 边沿 | 有目标但规划不出无碰撞轨迹 |
| | | `GLOBAL_TRJ_FAIL` | G9 | 边沿 | 前端全局轨迹失败 |
| `GOAL_REJECTED` | `planner` | `INVALID_DATA` / `Z_INVALID` / `TF_FAIL` / `DUPLICATE` | G7/G8 | 一次性 | **目标点本身非法，重试无意义** |
| `SAFETY_HOLD` | `planner` | `VEHICLE_IN_OBS` / `DEPTH_LOST` / `TRAJ_COLLISION` / `ODOM_INVALID` | G2/G4/G5/G6 | 边沿 | planner 已急停并锁住，控制权名义上仍在 planner |
| `ARRIVED` | `planner` | — | G12 | 一次性 | 轨迹执行完毕（P-D 到达确认） |

### 4.2 统一载荷骨架（每一帧都带状态快照）

不再区分「事件帧」和「状态帧」两套字段——所有消息同一副骨架，只换 `event`/`reason`/`detail`。这样**漏收一条事件不会丢语境**，下一帧心跳会把 `state` + `active` 补齐。

```json
// 1Hz 心跳（稳态）
{
  "event": "HEARTBEAT", "source": "planner", "node": "ego_planner_node",
  "ts": 1762000000123.0,
  "state": "EXEC_TRAJ", "reason": "OK", "detail": "",
  "goal_seq": "NA", "flight_epoch": "NA",
  "have_odom": true, "have_target": true, "have_trigger": true,
  "goal": "12.40,3.10,1.50", "dist_goal": 2.31, "alt": 1.52, "vel": 0.42,
  "lio": true, "traj_id": 128, "traj_dur": 1.90, "szd": false,
  "active": []
}

// 失效事件（同一骨架，快照字段照旧填）
{
  "event": "GOAL_UNREACHABLE", "source": "planner", "node": "ego_planner_node",
  "ts": 1762000000456.0,
  "state": "WAIT_TARGET", "reason": "OBSTACLE", "detail": "target=3.00,2.00,1.00",
  "goal_seq": "NA", "flight_epoch": "NA",
  "have_odom": true, "have_target": false, "have_trigger": false,
  "goal": "3.00,2.00,1.00", "dist_goal": 4.12, "alt": 1.52, "vel": 0.01,
  "lio": true, "traj_id": 0, "traj_dur": 0.0, "szd": false,
  "active": ["OBSTACLE"]
}
```

| 字段 | 来源 | 说明 |
|---|---|---|
| `state` | `exec_state_` 用 [L1037 的 `state_str[]`](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1037) 转字符串 | G11 全序（`WAIT_TARGET`/`EMERGENCY_STOP` 等）；兼作 `/stop_planning` 隐式回执（G13） |
| `reason` | 最近一次上报的 reason，无错 `"OK"` | 让只读心跳的消费方也拿得到主因 |
| `active` | §4.3 边沿集合的当前内容 | **late-join 自愈**：中途才开始订阅的消费方，下一帧就能重建「此刻仍成立的错误集合」，不依赖 `transient_local`/latched |
| `lio` | `have_odom_ && !grid_map_->getOdomDepthTimeout()` | §14 要求的 `lio` |
| `dist_goal`/`alt`/`vel` | `end_pt_ - odom_pos_`、`odom_pos_(2)`、`odom_vel_.norm()` | 到达确认（G12）与「不推进」判据 |
| `goal` | `end_pt_` 格式化 | 让消费方能对上「这条报错说的是哪个点」，比现在恒 `NA` 的 `goal_seq` 实用 |
| `detail` | 自由串 | 格式 `k=v` 空格分隔（`stage`/`trials`/`target`/`drone_id`），**不要塞嵌套 JSON** |

> `goal_seq`/`flight_epoch` 仍为 `"NA"`：planner 不生成 seq（§14 引.7 已记为未达成），本轮不强行补、不动协议；用 `goal` 坐标 + `ts` 做关联即可。

### 4.3 去重与判活规则

现有实现是单个 bool（[ego_replan_fsm.h#L89](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/include/ego_planner/ego_replan_fsm.h#L89)），只够一种错误态。P0 落地时换成「按 `reason` 键的边沿集合 `active_events_`」：

- 首次进入该错误态 → 发一条事件并记入集合；集合内容同时写进后续每一帧心跳的 `active`；
- 条件消失 → 从集合移除（下次再进入可再发）；
- `GOAL_REJECTED` / `ARRIVED` 是**一次性事件**，不进集合，每次命中直发（本身低频）；
- 收到 `/stop_planning`、或新目标点被接受 → 清空集合（新任务语境）；
- `HEARTBEAT` **不进集合、不计入任何不可达计数**，消费方第一道 `if` 就要拦掉。

**判活**：消费方按 `source` 分别记录最后收到帧的时间；`source=="planner"` 断流 >3s ⇒ `PLANNER_SILENT`（§14 H8 口径）。

### 4.4 为什么合流到一条话题是安全的（三条依据）

1. 消费成本为零：[`_mission_event_callback`](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/cruise_uav_task_manager.py#L172-L190) 首行就 `json.loads` + `event != ... → return`，1Hz 心跳只是多走两次字符串比较。
2. late-join 由 `active[]` 兜住，不需要给话题加 `transient_local`，也不需要新建 latched 服务。
3. 带宽可忽略：单帧 ~250B × 1Hz × 3 个发布源，比现有 100Hz 的 `planning/data_display` 小三个数量级。

---

## 5. 落地改动清单（逐项勾选）

### 5.1 ego 侧（包：`plan_manage`）

- [ ] `ego_replan_fsm.h`：删 `last_goal_obstacle_pub_`，改 `std::set<std::string> active_events_`；加 `mission_hb_timer_`、`last_reason_`、`last_state_`；加 `reportMissionEvent(event, reason, detail)`（内部拼装 §4.2 骨架）、`publishHeartbeat()`、`missionEventEdge(reason)`、`clearMissionEvents()`、`fsmStateName()` 声明。**不新增 publisher**，复用 `mission_event_pub_`。
- [ ] `ego_replan_fsm.cpp::init`：加 1Hz wall timer → `publishHeartbeat()`（参数 `fsm/mission_heartbeat_rate`，默认 1.0）。
- [ ] `changeFSMExecState`：`new_state != exec_state_` 时立即多发一帧心跳（保证 1Hz 粒度不丢 `EMERGENCY_STOP` 这类关键跃迁）。
- [ ] `publishMissionEvent` 重构为「快照 + 事件」统一拼装（现在的 `json_fields` 手工串改为内部拼 `state/reason/detail/active/...`），保持向后兼容：`event/source/ts/reason/goal_seq/flight_epoch` 字段名不变。
- [ ] `planNextWaypoint`（L377-387 / L437-440）：改走 `reportMissionEvent`；补 `UNOBSERVED`（`isUnknown`）与 `OUT_OF_MAP`（`getInflateOccupancy == -1`，见坑 1）；全局轨迹失败 → `GLOBAL_TRJ_FAIL`；成功后 `clearMissionEvents()`。
- [ ] `waypointCallback`/`triggerCallback`（L443-L572、L511-543）：非法数据/`z<-0.1`/TF 失败/重复忽略 → `GOAL_REJECTED`。
- [ ] `planFromGlobalTraj`（L1275-1282）：**补 G10 漏发**。
- [ ] `execFSMCallback`：`SEQUENTIAL_START`/`GEN_NEW_TRAJ`/`REPLAN_TRAJ` 三处失败 → `NO_PATH`（边沿去重）；`EMERGENCY_STOP` 恢复分支（L1244-1252）→ **补 G10 漏发**；轨迹执行完毕（L1202-1217）→ `ARRIVED`。
- [ ] `checkCollisionCallback`：`DEPTH_LOST`（L1349-1355）、急停分支 `TRAJ_COLLISION`（L1421-1427）、新增机体自检 `VEHICLE_IN_OBS`/`OUT_OF_MAP`（用 `odom_pos_`，20Hz 足够）。
- [ ] `odometryCallback`（L727-L793）：`ODOM_INVALID`（边沿去重，避免逐帧刷屏）。
- [ ] `callReboundReplan` 成功（L1451）：清除 `NO_PATH`/`GLOBAL_TRJ_FAIL` 边沿。
- [ ] `stopPlanningCallback`（L931-944）：清空 `active_events_` + 立即发一帧心跳（`state:"WAIT_TARGET"`，即 G13 回执）。

### 5.2 编排侧（包：`task_dispatcher`）

- [ ] [`_mission_event_callback`](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/cruise_uav_task_manager.py#L172-L190) **入口先分流**：`event=='HEARTBEAT'` → 只更新 `last_planner_hb_ts`/`planner_state` 后立即 return；`event=='GOAL_REJECTED'` → 走 §6 立即失败分支；其余按 reason 白名单进 §6 矩阵。
- [ ] 去重窗从单个 `last_obstacle_ts` 改成 `dict[(source,event,reason)] → ts`。
- [ ] 新增 `SAFETY_HOLD` 分支：不进「同点重试」计数，直接 `RETURN_INIT`（`trigger` 用 reason）；`vel<0.1` 久持 → `EMERGENCY_LAND`。
- [ ] 新增 `GOAL_REJECTED` 分支：**立即失败该航点且不重试**，`taskProgress=3` + 原因为参数/坐标错误（区别于环境不可达，避免无谓的 3 次重试拖住航线）。
- [ ] 判活：`source=='planner'` 心跳断流 >3s → `PLANNER_SILENT`；在 [`perform_action_walk_to_pos_3d` 的内层循环](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/perform_action_manager.py#L306-L355) 加两条早退：① `PLANNER_SILENT`；② 收到针对当前目标（用 `goal` 坐标匹配）的 planner 硬错误 → 立即返回 `GOAL_UNREACHABLE:` 前缀，**不再等满 5s 停滞窗口**。
- [ ] 心跳快照的存放位置建议放 [`ros2_topic_subscriber.py`](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/ros2_topic_subscriber.py)（已是集中的话题读取入口，perform_action 与 cruise 共用），**不要再为该话题建第三个订阅者**；cruise 现有的那一个订阅把快照转存进去即可。
- [ ] 自环防护（合流后风险变大）：cruise 自己既 publish 又 subscribe `/mission_event`（[L139-L145](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/uav_service/src/task_dispatcher/task_dispatcher/cruise_uav_task_manager.py#L139-L145)）。若编排侧也补 1Hz `HEARTBEAT{source:"executor"}`，**判活必须锁死 `source=='planner'`**，否则自己的心跳会把 planner 失联永远掩盖掉。

---

## 6. 编排侧消费矩阵

| `/mission_event` 的 event / reason | 严重度 | 处置（映射到现有状态机） |
|---|---|---|
| `HEARTBEAT` | — | **只更新快照与判活时间**，绝不进计数 |
| `GOAL_UNREACHABLE/OBSTACLE`、`/NO_PATH`、`/GLOBAL_TRJ_FAIL` | 中 | 进 `same_point_retry`（3 次→`GOAL_SUBSTITUTED` 跳点；连续 2 点→`RETURN_INIT`）——与现状一致 |
| `GOAL_UNREACHABLE/UNOBSERVED` | 中 | **先不跳点**：原地等待感知补全（有限次，如 1×2s）后再计入重试；遮挡≠不可达 |
| `GOAL_UNREACHABLE/OUT_OF_MAP` | 高 | 直接 `RETURN_INIT`（航线整体出图，跳点无意义） |
| `GOAL_REJECTED/*` | 高 | 当前航点立即失败、**不重试**；上报云端 `taskProgress=3` 并附 reason |
| `SAFETY_HOLD/VEHICLE_IN_OBS`、`/DEPTH_LOST`、`/ODOM_INVALID` | **最高** | 地图与定位都不再可信 → 立即 `RETURN_INIT`；若 `vel<0.1` 且持续 → 走 `RETURN_BLOCKED` 迫降 |
| `SAFETY_HOLD/TRAJ_COLLISION` | 高 | 记 1 次「险遇障」；同航点 ≥2 次 → 视同不可达走跳点 |
| `ARRIVED` | — | 替代纯距离判据做到达确认（§14 P-D） |
| `state` 快照 = `EMERGENCY_STOP` | 高 | 视为 planner 已锁机：停止刷新 goal，等 `SAFETY_HOLD` reason 定性 |
| `state` 快照 = `WAIT_TARGET`（在 `/stop_planning` 之后） | — | 控制权已交回飞控，可安全下发原生 `LAND`/`RTL`（G13 回执） |
| `source=planner` 断流 >3s | **最高** | `PLANNER_SILENT`：`/stop_planning` + 交回飞控原生 `RTL`/悬停；**不得继续按「飞行中」上报** |

---

## 7. 已知坑（会影响后续同步，务必先读）

1. **`getInflateOccupancy` 出图返回 `-1`，被 `if(...)` 当成「有障碍」**（[grid_map.h#L383-L392](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_env/include/plan_env/grid_map.h#L383-L392)）。现有 4 处判据（[L378](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L378)、[L1244](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1244)、[L1275](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1275)、[L1368](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1368)）全部是 `if (getInflateOccupancy(...))`，因此**「目标点在图外」现在被误报成「目标点在障碍里」**。新增 `OUT_OF_MAP` 时必须按 `== -1 / == 0 / > 0` 三态写。
2. **100Hz 空转**：`GEN_NEW_TRAJ`/`REPLAN_TRAJ` 失败后原地重入同一状态（[L1162-L1165](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L1162-L1165)），没有放弃计数也没有超时。上报若不做边沿去重会直接刷爆话题与日志。
3. **同一条「目标在障碍」判据分布在 4 处，只有 2 处上报**（G10）。改的时候若只动一处，编排侧计数仍会随机漏。
4. **`planning/data_display` 不是状态口**：字段是无名 `a~e`，任何消费方都无法稳定解释，不要往上面加语义。
5. **不新增 `.msg`**：`traj_utils/msg/*` 一改就要重新生成消息包并同步真机侧全部依赖方（`rosmsg_tcp_bridge`、`bridge_node` 也依赖它）。String+JSON 与现有事件通道同构。
6. **日志文案与实际订阅不一致**（[xtd2_traj_server.cpp#L894](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/xtd2_traj_server.cpp#L894)）——现场排查以代码为准。
7. **planner 报错 ≠ 一定收到过 goal**：goal 在上游 comms 闸门就被静默丢弃时，planner 侧不会产生任何事件。因此「没有报错」不能推断「在正常飞」——要靠 `HEARTBEAT` 有没有到、`state` 是不是 `WAIT_TARGET` 来定性。
8. **合流后新增的一条**：心跳与事件同话题 ⇒ 消费方必须在**第一道 `if`** 就按 `event` 与 `source` 双字段分流。两个具体风险：① 心跳被当成事件计数（`HEARTBEAT{reason:"OBSTACLE"}` 仍带主因，1Hz 会把 `same_point_retry` 秒冲到跳点阈值）；② 编排侧自己的心跳把 planner 失联掩盖。现有 `publishMissionEvent` 只在异常路径被调用，所以**心跳帧必须单独走 `publishHeartbeat()`，且它填的 `reason` 只作展示、不得进入任何计数器**。
9. 队列深度维持现状 10（[L128](file:///home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_third_party_pkgs/motion_planning/ego-planner-swarm/src/planner/plan_manage/src/ego_replan_fsm.cpp#L128)）：1Hz 心跳 + 边沿事件远不会溢出，**不要为心跳去调 QoS**。

---

## 8. 验证方法

```bash
# 1) 心跳与事件是否都活着（合流在一条话题上）
ros2 topic echo /mission_event
ros2 topic hz /mission_event            # 基线≈1Hz（仅 planner 心跳）；编排侧若也补心跳则 2~3Hz，事件突发另加
#   `ros2 topic echo` 没有 --filter，按源粗筛用 grep：
ros2 topic echo /mission_event | grep --line-buffered '"source":"planner"'

# 2) 故障注入（离线仿真，链路见 uav_service/src/task_dispatcher/launch/offline_route_sim.launch.py）
#    a. 目标点打进墙里 → 期望 GOAL_UNREACHABLE/OBSTACLE 1 条 + 后续心跳 active:["OBSTACLE"]
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {stamp: now, frame_id: world}, pose: {position: {x: 3.0, y: 2.0, z: 1.0}, orientation: {w: 1.0}}}"

#    b. 图外目标 → 修复坑 1 后应报 OUT_OF_MAP；修复前会被误报成 OBSTACLE
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {stamp: now, frame_id: world}, pose: {position: {x: 999.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}"

#    c. 下发 z=-1 的 goal → GOAL_REJECTED/Z_INVALID（当前：无声吞掉，编排侧要等满 5s 停滞窗）
#    d. 掐掉 grid_map/cloud 发布者 >odom_depth_timeout → SAFETY_HOLD/DEPTH_LOST + 心跳 lio:false
#    e. 让飞机悬停贴进障碍膨胀区 → SAFETY_HOLD/VEHICLE_IN_OBS（当前版本完全无输出）
#    f. 手动发 /stop_planning → 应立刻收到一帧 HEARTBEAT{state:"WAIT_TARGET"}
#    g. kill ego_planner_node → 编排侧应在 3s 内报 PLANNER_SILENT（且 executor 自己的心跳不能抑制它）
```

编排侧联调看 `cruise_uav_task_manager` 日志里 `[mission_event] 收到 ego ... 上报` 的计数是否随注入递增、`same_point_retry` 是否在不满足 5s 停滞窗的情况下就跳点，以及**注入 a 之后 1Hz 心跳没有把计数继续推高**（验证坑 8）。

---

## 9. 与仓库根设计文档的差异（同步时要一并改的地方）

| 条目 | 设计文档现说法 | 本文口径 |
|---|---|---|
| `/mission_status` | §14 把它列为独立话题（`String+JSON，1Hz 稳态+事件即发，字段 goal_seq/state/reason/dist_goal/alt/battery/lio/ts`），引.5#5 标注「单出口仍 ⏳」 | **本方案改为不新增话题**：心跳以 `HEARTBEAT` 事件形式走 `/mission_event`，字段沿用 §14 那套（去掉 `battery`——电量归 comms；去掉 `alt` 的歧义，明确 `alt=odom.z`）。需同步修订设计文档 §14 与引.5#5，否则两边口径会分叉 |
| 心跳字段 `goal_seq` | §14 要求携带 | 仍为 `"NA"`：planner 无 seq 概念，用 `goal` 坐标 + `ts` 关联（引.7 的「字段位有、值恒 NA」这条不改口径） |
| `SAFETY_HOLD` / `GOAL_REJECTED` / `ARRIVED` | §14 引.7 标注「无任何代码命中」 | 本文把这三个落为 planner 侧事件，**不新增事件名**，可据此把该条从 ⏳ 提为 ◐ |
| `reason` 枚举 | §14 表只写 `STALLED/OBSTACLE` | 需扩为 §4.1 全集（`UNOBSERVED/OUT_OF_MAP/NO_PATH/GLOBAL_TRJ_FAIL/VEHICLE_IN_OBS/DEPTH_LOST/TRAJ_COLLISION/ODOM_INVALID/INVALID_DATA/Z_INVALID/TF_FAIL/DUPLICATE`） |
| 航点失效链「未经测试」 | 引.7-A9 | 本文 §7-1/-3 给出两条**静态即可判定**的实现缺陷（图外误报、漏发上报），建议先修再测 |
