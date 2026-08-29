# 07 新导航栈：CMU planner suite（terrain analysis + FAR planner + local planner）

更新时间：2026-08-29。本文件记录为替代 Nav2 2D 导航而引入的新规划栈，
来源：https://github.com/Quadruped-dyn-insp/Go2_planner_suite（MIT，纯 CPU）。

## 为什么换

- 原方案是 3D 感知 + 2D 投影导航（pcd_to_nav_map + Nav2），室外坡道/
  台阶/负障碍处理不了，巡检要室内外混跑。
- 新方案 terrain analysis 输出高程图 + 可通行性，FAR planner 在线建
  可见图做全局规划，local planner 反应式避障。不需要 PGM 栅格地图。

## 现有改动清单

| 位置 | 内容 |
|---|---|
| `ros2_ws/src/terrain_analysis` | 地形分析（局部 4m），来自 Go2_planner_suite |
| `ros2_ws/src/terrain_analysis_ext` | 地形分析扩展（40m，供 FAR planner） |
| `ros2_ws/src/far_planner` | FAR 全局规划器（far_planner_updated 版，带坡度） |
| `ros2_ws/src/visibility_graph_msg` | far_planner 依赖的消息包 |
| `ros2_ws/src/local_planner` | 局部规划器 + 路径跟随器 |
| `config/far_planner_go2.yaml` | FAR planner GO2 参数（world_frame=camera_init） |
| `config/planner_stack.launch.py` | 一键启动，mode=sensing/full |
| `scripts/inside_planner.sh` | 容器内入口 |
| `scripts/run_planner_stack.sh` | 宿主机入口 |

已编译进 ros2_ws（install 目录），全部通过无传感器冒烟测试（2026-08-29）。

## 话题接线（关键）

新规划栈全部 remap 到现有 FAST-LIO 输出，FAST-LIO 一行不改：

```text
FAST-LIO /Odometry              -> /lidar_odometry/pose, /odom_world
FAST-LIO /cloud_registered_body -> /lidar_odometry/deskewed_scan_points, /scan_cloud
terrain_analysis                -> /terrain_map
terrain_analysis_ext            -> /terrain_map_ext
far_planner 订阅 /goal_point   -> 输出 /way_point -> local_planner
local_planner pathFollower      -> /cmd_vel（注意：尚未接入运动桥！）
```

## 移植中修的坑

1. far_planner 的 package.xml 引用了不存在的 `srv_msgs`（代码未用），已删。
2. `local_planner/paths/correspondences.txt` 上游忘提交，localPlanner 启动即退出。
   已用 `paths/gen_correspondences.py` 按 path_generator.m 逻辑重新生成
   （72611 行，容器内有 numpy，宿主机没有）。
3. 本机 5.7GB 内存：colcon 必须 `--parallel-workers 1` +
   `CMAKE_BUILD_PARALLEL_LEVEL=2`，否则 OOM 被杀。

## 上电实测记录（2026-08-29）

第一、二阶段均已完成：

1. **sensing 模式**：terrain 节点稳定，/terrain_map 2Hz，CPU 合计 ~45%，
   内存 81MB。**P0 近场假障碍验证通过**：清空前方杂物后，前方 2.5m 内
   terrain 障碍点为 0 个；机身近场回波（0.15m）被归类为可通行
   （低 intensity），不再产生假障碍。左右扇区障碍距离与实际杂物一致。
2. **full 模式**：goal_point -> far_planner -> /way_point -> localPlanner
   （PATH FOUND）-> pathFollower -> /cmd_vel 全链路打通。
3. **实测修的两个问题**：
   - terrain_analysis 输出 frame_id 硬编码 "map"，far_planner TF 查找失败。
     launch 里已加 map->camera_init 恒等静态 TF。
   - pathFollower 缺 autonomyMode 参数时速度输出恒为 0，已补全参数集。
4. **遗留关键问题**：pathFollower 当前输出纯旋转指令（angular.z=-0.52，
   路径方向与朝向差约 116°）。GO2 固件不能原地旋转 -> 这成为下一个
   必须解决的问题，否则路径跟随会卡在"等待转向"。
5. 排障经验：分析 terrain_map 必须用完整四元数变换（MID360 安装俯仰
   35.6°，只用 yaw 会把点放错位置）。探针脚本在
   `runtime/probe_terrain.py`。

## GO2 旋转问题排查记录（2026-08-29，已解决）

实测脚本：`runtime/rotation_walk_probe.py`（三组对照）、
`rotation_verify.py`（真值校准）、`rotation_settle_probe.py`（稳定期验证）、
`rotation_bisect.py`（ID/节奏二分）、`test_rotate_bridge.py`（端到端验收）。
**注意：前两个脚本的"积分转角"打印有单位 bug（弧度标成度），以换算值为准。**

### 固件行为结论

1. **纯旋转 Move(0,0,z) 被固件完全忽略**。
2. **弧线 Move(vx≥0.10, 0, z) 流式可靠**：Move(0.10,0,0.5) 每 0.5s 流式
   6 秒，IMU 真值转角 +153°，前进仅 0.40m（转弯半径 ~0.15m，等效原地转）。
3. **站立后约 10 秒内 Move 基本被忽略**（稳定期）。所有"时灵时不灵"
   的现象多与测试时站立时长有关。
4. **关键二分结论**：请求 ID 用大数值（时间戳×10⁶）时，**不带 Stop 帧
   的连续流式会被无视**；同样的大 ID 配脉冲节奏（Move 0.3s → Stop →
   间隔 0.12s，即已验证的 web 走路节奏）可靠（63°/4s）；小 ID 连续流式
   也可靠（101°/4s）。
5. mode/gait_type 字段全程为 0（含行走时），对判断无帮助。

### 最终方案（已实现并验收）

`patrol_bridge/go2_state_bridge.py` rotate_only 分支：
- 检测到 rotate_only（|z|>0.005 且 x、y≈0）时，把参数改写为
  弧线 `{"x": ROT_ARC_X(默认0.10), "y":0, "z": 限幅±1.0}`，
  然后**落回已验证的脉冲节奏路径**（与走路共用同一节奏）。
- `ROT_ARC_X` 可用环境变量 `GO2_ROT_ARC_X` 调整。
- **端到端验收通过**：走 rotate_only 通路（同网页 Q/E 键），6 秒实测
  +86.8°（平均 0.25 rad/s），无安全拦截。
- 新规划桥 `scripts/planner_motion_bridge.py` 的旋转指令直接透传
  x=0（状态桥自动改写为弧线），`GO2_ROT_MODE=skip` 可丢弃旋转。

### 遗留观察项

- 弧线旋转带 0.10 m/s 前进分量：goal 处转向会画小圈，`noRotAtGoal: True`
  已在 pathFollower 里抑制到点后的转向，实际巡检影响待观察。
- 每次站立后首条指令前等待 ≥10s：巡检流程是"站立→长时间作业"，
  不受影响；网页手动控制连按即可。

## 怎么跑（上电后）

```bash
# 第一阶段：只看地形分析输出（狗趴着不动也行）
PLANNER_MODE=sensing ./scripts/run_planner_stack.sh
# RViz 看 /terrain_map、/terrain_map_ext；同时看 CPU/内存

# 第二阶段：全套（此时 /cmd_vel 无人转发，狗不会动）
PLANNER_MODE=full ./scripts/run_planner_stack.sh
# RViz 发 /goal_point (PointStamped, camera_init 系) 看规划
```

注意：充电期间测试用的是 `runtime/cyclonedds_wifi.xml`（有线网卡断开），
正式跑时 docker-compose 默认的 cyclonedds.xml（有线口）不受影响。

## 还没做（按顺序）

1. 上电实测：terrain_map 质量、CPU/内存占用、0.4m 假障碍是否消失。
2. `nav_motion_bridge.py` 适配 local_planner 的 /cmd_vel（现在只认 Nav2
   的 /plan 和 DWB 速度；安全联锁逻辑保留）。
3. 网页/Blockly：巡查点下发改 /goal_point，路线显示改订阅新路径话题。
4. 全局重定位（FAST_LIO_LOCALIZATION）与 GO2 原地旋转问题，独立课题。

## 全链路联调记录（2026-08-29 傍晚，狗充电暂停）

### 系统链路（当前实际形态）

```text
MID360 -> livox driver -> FAST-LIO2 -> /Odometry + /cloud_registered(_body)  [域42]
       -> terrain_analysis(+ext) -> /terrain_map(/_ext)
       -> far_planner -> /way_point -> localPlanner -> /path
       -> pathFollower -> /cmd_vel
       -> scripts/planner_motion_bridge.py（规划容器内，域42）
       -> runtime/go2_nav_command.json
       -> go2_state_bridge（域0，弧线旋转+脉冲节奏）
       -> GO2 固件
```

### 傍晚验证结果

1. **端到端已打通**：第三次实测中狗自主走过 0.37m（goal→规划→桥→狗，
   发布计数 44→172），链路每一环都确认工作。
2. **far_planner 三个隐藏门槛**（全部踩过）：
   - `/start_far_planner` 服务不调用则规划循环不跑（已写入 inside_planner.sh 自动调用）；
   - **`is_stop_update_` 默认 true，可见图更新默认冻结**，必须调
     `/resume_visibility_graph_update`（已写入 scripts/start_far_service.py 自动调用）；
   - 全空旷场地建不了可见图（节点来自障碍轮廓），场地里要有至少 2-3 个物体。
3. **参考系（重要发现）**：robot_state.json 的 pose 是 map_level 系
   （localization_alignment.json 偏移换算），而 /path、/goal_point、
   /Odometry 都是 camera_init 系。规划桥的路径起点校验必须用
   /Odometry，否则永远差 5.4m 拒绝一切。
4. **旋转在导航中的表现**：dirDiff>120° 时 pathFollower 走旋转分支，
   弧线 workaround 生效（实测转了 115°），但每 0.1m/s 前进分量有漂移，
   加上 arc 前进，狗曾进入中间路点 0.5m 圈被判"到达"而提前停。

### 当前卡点（按优先级）

1. **稳定性（根因）**：5.7GB 内存。FAST-LIO 挂死过一次、far_planner
   OOM 三次。已加 4GB swap（/swapfile2，共 6GB），**待验证效果**。
   根治：地图无界增长（filter_size_map 已调 0.5）+ 迁移 Orin NX 16GB。
2. **走停抖动**：pathFollower 速度输出间歇归零（狗走两步停一步）。
   疑点：路径起点校验在阈值边缘（0.35→0.6 已放宽，仍偶发）；dirDiff
   大时频繁进入旋转分支。需要加日志定位。
3. **待重测**：正前方 1m 净空直线导航（nav_to_goal_test2.py 1.0 已就绪）。

### 狗充电完恢复步骤

```bash
# 1. 上电后确认在线：ping 192.168.123.170（雷达）、192.168.123.161（狗）
# 2. 起建图栈（若没跑）：
cd ~/go2_mid360_stack && nohup ./scripts/run_mid360_nav_mapping_stack.sh &
# 3. 起规划栈（自动调 start_far + resume_vgraph 服务）：
nohup env PLANNER_MODE=full ./scripts/run_planner_stack.sh &
# 4. 规划容器内起桥：
C=$(docker ps -q | head -1)
docker exec -d $C bash -c 'source /opt/ros/humble/setup.bash; source /project/ros2_ws/install/setup.bash; python3 /project/scripts/planner_motion_bridge.py >>/project/runtime/logs/planner_motion_bridge.log 2>&1'
# 5. 一米导航测试（站立+发目标+监控+自动趴下）：
docker exec $C bash -c 'source /opt/ros/humble/setup.bash; source /project/ros2_ws/install/setup.bash; python3 /project/runtime/nav_to_goal_test2.py 1.0'
```

注意：docker exec 长命令可能触发笔记本 sshd 限流，连续 SSH 间隔 20-60s。

## 1米直线导航联调实录（2026-08-29 晚，第3轮通过）

### 结论
狗在规划器全自主控制下走满 1 米目标（最大位移 0.75m，随后撞到前方约 1m 处的玻璃门，
人工遥控介入拉回）。连续行走段 8~10 秒稳定，指令发布 300+ 条，走走停停仍轻微存在
（pathFollower 循环偶被 CPU 饿到，5.7GB 内存瓶颈，预期 16G NX 解决）。

### 本轮连环修复（共5项）
1. **/path 话题撞名（最隐蔽）**：FAST_LIO laserMapping.cpp:935 也发布 nav_msgs/Path
   叫 /path（里程计轨迹），与 localPlanner 规划路径冲突。调试时把轨迹当路径分析。
   修复：mapping.launch.py 加 remappings=[('/path','/lio_trajectory')]。
2. **自体阻挡**：localPlanner obstacleHeightThre=0.15 把狗自己后腿/躯干(0.17~0.34m高)
   当障碍，所有方向 7~10cm 即碰撞，路径原地打转（趴着最严重，站立也有）。
   修复：obstacleHeightThre 0.15→0.35。运动桥 0.75m 前向走廊（阈值0.36）独立兜底。
3. **Laser_map CPU 黑洞**：累积图 400万+ 点每周期序列化，pathFollower 主循环一度
   30 秒一轮，状态桥 0.3s 收不到新指令即停。修复：导航阶段 fastlio_mid360.yaml
   publish.map_en: false（建图会话需改回 true！）。
4. **dirDiffThre 过严**：0.1rad(5.7°) 导致路径稍偏即停下先转向（圆弧转向代价高）。
   修复：0.1→0.35rad，边走边修正。
5. **运动桥路径校验错误**：本套件 localPlanner 的 /path 是 base_footprint 车体坐标
   （恒以 (0,0) 开头），旧端点-vs-世界里程计校验在狗离开 FAST-LIO 原点后必然误拒。
   修复：改为路径非空+新鲜度（last_plan<3s）校验。

### 关键测量数据
- 模板路径：startPaths.ply 仅 7 组×101 点，半径 1.0m，点距 1cm；发布时 pathScale=1.25
- 路径裁剪 = min(pathRange, relativeGoalDis)/pathScale，正确裁剪到目标距离
- pathFollower stopDisThre=0.2：路径短于 0.2m 会直接判定到达（早期抽搐主因之一）

### ⚠️ 玻璃门盲区（重要教训）
激光 905nm 打玻璃穿透/镜面反射，几乎无回波 → localPlanner、运动桥走廊、地形图
三层防护对玻璃同时失效，狗以 0.2m/s 撞上玻璃门。应对：
- 巡检点位标定避开玻璃正对直线；可在地图/点位系统加禁行区标注
- 硬件补盲（超声/ToF）或 C12 可见光玻璃检测（远期）
- 遥控介入优先级正常，人工拉回有效

### 状态备注
- swap 已扩至 9GB（/swapfile3 4GB，重启失效需重新 swapon，fstab 未写入）
- 备份：backups/mapping.launch.py.bak_20260829、planner_stack.launch.py.bak_20260829、
  fastlio_mid360.yaml.bak_20260829_mapen

## 网页直连导航链路打通 + 模式分离（2026-08-29 深夜）

### 架构决定
建图服务 / 导航服务分离（用户拍板，后期独立部署）。感知层（MID360驱动+FAST-LIO+
patrol_bridge）是公共底座，建图/导航是上层模式：
- **建图模式**：map_en=true（需要累积图保存），单独跑
- **导航模式**：map_en=false（Laser_map 是 CPU/内存黑洞），网页实时点云改由
  patrol_bridge 的 **/cloud_registered 滚动窗口**（10s，每帧抽稀到4000点）生成
  cloud.json（source=scan_window），选点/导航全程内存安全

### 网页「启动导航→导航到此」全链路（本轮打通）
1. /api/navigation/start → run_planner_stack.sh（默认已改 full），live-session 模式
   允许无匹配地图导航（旧地图会话不匹配时自动降级，不再拦截）
2. inside_planner.sh full 模式自动拉起 planner_motion_bridge + goal_relay
3. **goal_relay.py**（新增）：goal.json(map_level) → /goal_point(camera_init)，
   变换 = 固定雷达安装角(roll/pitch) + localization_alignment.json(x,y,z,yaw)，2Hz 持续发布
4. /api/navigation/goal 支持 __live__ 会话点位（session_id 校验）

### 修复清单
- **tmp 文件竞态**：server.py write_json 与运动桥共用 .tmp 名互相踩（ENOENT 报错）
  → 临时名加 pid
- **路径显示错位**：localPlanner /path 是 base_footprint 车体系 → 运动桥做
  车体→camera_init(odom)→map_level 两级变换后写 nav_path.json
- **遥控转弯**：teleop(小ID连续流)支持纯旋转（二分实测+101°），不再吃导航的圆弧
  改写；vyaw 上限 0.5→1.0 rad/s；网页速度档改 0.2/0.30/0.60（默认0.60）
- **定位保护永久锁死**：狗趴下/站起的雷达垂直移动触发位姿突变保护后永不恢复
  → 增加 3 秒稳定自动恢复
- **pathFollower 晃动**：yawRateGain 4→2.5、stopYawRateGain 10→5（待验证）
- **点不可见**：/api/checkpoints/save 标签条件放宽为 robot online（原来要求建图
  服务运行中，间隙选点变成无标签孤儿）

### ⚠️ 注意
- docker restart 会移除 --rm 容器（又一次踩坑）——重启感知/状态桥一律 docker stop +
  脚本重拉
- 网线（笔记本↔狗底座）承载雷达 192.168.1.x + 狗 192.168.123.x，断开 = 定位/狗控
  全断，DDS 报 ddsi_udp_conn_write failed 即此因
- go2_state 服务经脚本手动重拉后网页服务状态可能显示停止（仅显示问题，功能正常）

### 下次开机恢复步骤
1. 笔记本 Wi-Fi 就绪 → 网页强刷
2. 「建图与点位」页 → 狗「站立」→ 点云选点
3. 「导航控制」页 → 「启动导航」→ 「导航到此」
