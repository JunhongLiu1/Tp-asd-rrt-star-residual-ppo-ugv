# 在线风险感知导航重构进度

最后更新：2026-08-22

## 已完成

1. 建立 C++ TP-ASD-RRT* 规划核心，支持风险区域采样、目标/均匀混合采样、
   联合风险负梯度 APF、停滞自适应、超时、取消、确定性随机种子和结构化失败。
2. 建立在线规划节点、路径指标与 CSV 记录节点。
3. 将辐射真值与在线估计隔离：
   - `/ground_truth/radiation_map` 仅提供评估真值和栅格几何；
   - `/radiation/dose_rate_usv_h` 提供局部传感器测量；
   - `/risk_map` 提供在线估计图；
   - `/risk_map/metadata` 提供版本、覆盖率和观测单元数。
4. 在线风险图初始为未知，不再复制完整真值；未知区域按可配置保守风险参与规划。
5. 跟踪器只发布 `/control/base_cmd`，Safety Gate 独占最终 `/cmd_vel`。
6. Safety Gate 支持急停、命令超时、限速、限加速度、NaN/Inf 拦截，并默认禁止倒车。
7. 路径跟踪器已升级为 Pure Pursuit：
   - 沿路径弧长选择前视点；
   - 动态前视距离；
   - 机器人坐标系曲率；
   - 弯道降速和终点减速；
   - 路径偏离停车；
   - 前视点在车后时先原地对准。
8. Pure Pursuit 数学已提取为独立 `pure_pursuit_core` 库。

## 当前验证状态

- `risk_aware_planner_cpp` Release 编译成功。
- 本包自动测试：45 tests，0 failures；新增 planner-node edge-cost 分量测试确认
  terrain/radiation/time 单因素成本单调，关闭 time penalty 只移除 time 项；工作区另有 3 组既存的
  `radiation_mapping` Python 代码质量失败（copyright/flake8/pep257），与 PID 无关。
- Pure Pursuit 独立测试覆盖动态前视、直行、左右转向、反向初始姿态、终点停车和退化目标。
- TP-ASD 30-seed 合成配对 A/B（31--60）三图均 30/30 成功，risk/guided
  计数非零且 risk-wall 触发 adaptation；相对 **legacy risk-weighted RRT* baseline**，
  open cost -0.1975366，risk-wall +0.0973630，gradient +0.0189667，并在三图均
  使用更少平均节点。故结论是“机制通过”，不是“所有场景性能更优”。证据位于
  `acceptance_logs/tp_asd_20260822/`。
- 本次自适应机制合入前的 M1--M3、PID、RL 路径结果统一标记为
  **legacy risk-weighted RRT* baseline**；当前默认启用实现才称 TP-ASD-RRT*。
- Gazebo 正式入口已切换到 r3 world，真值图发布到
  `/ground_truth/radiation_map`，不再与在线 `/risk_map` 混用。
- M2 在线链路已在 Gazebo 中通过：地形、真值辐射、在线风险图、里程计均就绪，
  规划器持续返回成功路径（本次短直线规划约 0.08--0.19 s）。
- M3 短直线闭环已在单一干净 Gazebo 栈中通过：
  - `/control/base_cmd` 与最终 `/cmd_vel` 均只出现正线速度，峰值约 0.0288 m/s，反向命令为 0；
  - Pure Pursuit 连续处于 `TRACKING`，随后进入 `GOAL_REACHED: zero command`；
  - rosbag 最后 2 s 的最终线速度最大值为 0，确认到站停车；
  - 验收证据位于 `acceptance_logs/m3_20260822/m3_single_stack_bag/`。
- 修复在线辐射安全语义混用：`dose_replan_threshold=0.5` 只触发重规划，
  独立的 `dose_stop_threshold=8.0` 才触发紧急停车，消除了 `/e_stop` 抖动。
- PID 前 Gazebo/运行时验收已全部完成：左右转向、反向初始朝向原地对准、
  终点减速停车、路径偏离停车、odom 中断停车、控制命令超时、辐射急停，
  以及 `enable_motion=false` 零速度基线均有 rosbag 证据。
- PID 阶段已启动：
  - 新增独立双通道速度 PID 核心，具备积分/输出限幅、抗积分饱和和异常复位；
  - Pure Pursuit 负责参考速度，线速度与角速度 PID 使用 odom 反馈闭环修正，输出继续经过 Safety Gate；
  - 停车、失联、偏离、急停和原地对准分支会复位相应 PID 状态；
  - 首次 Gazebo 短直线验收通过：152 个 PID 跟踪周期、无倒车、到站后最后 20 条最终命令为零；
  - 验收证据位于 `acceptance_logs/pid_20260822/pid_short_line_bag/`。
- PID 工程加固已完成：执行器实际余量参与抗积分饱和，异常控制周期会复位 I/D，
  默认微分增益冻结为零；主在线辐射 launch 可覆盖 PID 开关与六个增益。
- Foxy 参数覆盖问题已修复并由运行日志确认 PID 可切换为 `DISABLED`。测试启动链的
  同名文件遮蔽问题也已定位，并新增 readiness probe 阻止输入未就绪时产生伪数据。
- 严格 A/B 已完成：当前 PI 比关闭 PID 到达略快、误差略小，无倒车且终点稳定停车；
  约 17% metrics 周期触发限幅。增益冻结为 linear `0.80/0.10/0.0`、angular
  `0.50/0.05/0.0`，限幅率保留为后续安全监控项。
- 受限残差 PPO 阶段已完成首轮工程闭环：
  - 新增独立 `risk_aware_residual_rl` 包；策略只对冻结 PID 输出施加有界残差，
    Safety Gate 仍是 `/cmd_vel` 唯一最终发布者；
  - Torch/SB3 推理运行在隔离子进程，具备硬超时、进程组清理、故障回退 PID 和
    锁存关闭；checkpoint 必须通过 SHA256 allowlist 与观测/动作合同审计；
  - 50k PPO 在 100 个未见 surrogate seeds 上 100% 到达，平均 196.47 步、回报
    -28.02；同 seed 零残差为 269.15 步、-62.41，全部预注册离线门槛通过；
  - 唯一一轮非零策略 Gazebo 短目标 smoke 通过：worker 正常、残差有界、无倒车，
    约 7.8 s 到达并终停，无急停/超时/锁存；证据位于
    `acceptance_logs/rl_ppo_20260822/learned_gazebo_smoke_valid/`；
  - 该结果仅为 Gazebo 集成 smoke，不构成实体机器人部署授权。
- 验收脚本已补齐 hard 地形服务与 `/e_stop=false` 监督心跳；后台运行时应重定向
  Gazebo 控制台，避免高频地图日志造成前台管道背压。

## 下次从这里继续

PID 与受限残差 RL/PPO 的首轮代码、离线评估和 Gazebo 集成 smoke 已完成。下一阶段若
面向部署，需要多目标、多地形、未见辐射分布和完整故障矩阵的统计评估；当前 checkpoint
仍保持“非部署授权”。
