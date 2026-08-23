# TP-ASD 独立验收报告

日期：2026-08-22。测试对象为 `risk_aware_planner_cpp` 当前实现，未修改实现代码。

## 构建、单测与启动检查

- Release build：`colcon build --base-paths src --packages-select risk_aware_planner_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release`，通过。
- planner 全部 6 个 CTest suite、45 tests：通过（14 core、edge cost、pure pursuit、safety gate、risk map、PID；`100% tests passed, 0 failed`）。
- planner core 机制测试：14/14 通过。
- planner node 使用的 edge-cost 加权函数已提取并测试：terrain、radiation、time
  单因素增大时总成本严格增大；`include_time_penalty=false` 时只去除 time 项；
  非有限分量 fail closed。
- 三个 planner launch 文件 `py_compile`：通过；`experiment` 与 `online_radiation` `--show-args` 可见 `enable_adaptive_sampling`。
- README/REBUILD_PROGRESS 已标记 TP-ASD、M1-M3/PID/RL 状态；默认 `enable_adaptive_sampling=true`，可显式关闭恢复 legacy 采样路径。

## 固定合成地图

`synthetic_ab_30seeds.json` 保存 30 个配对 seed（31--60）、open/risk-wall/gradient 三图、legacy/adaptive 汇总与采样计数。这里 `legacy` 明确标为
**legacy risk-weighted RRT* baseline**，`adaptive` 标为 **TP-ASD-RRT***。
三图两模式均 30/30 成功；adaptive 的 risk/guided 计数非零，wall 触发
adaptation。open 平均 cost 低 0.1975366；risk-wall 高 0.0973630；gradient
高 0.0189667。核心单测覆盖低风险偏置、风险梯度方向、goal attraction、
停滞自适应、同 seed 确定性、关闭开关 legacy RNG/不读 risk、坏 callback
均通过。成本并非所有地图都下降，因此这是机制验收，不是性能优越性声明。

## 代价与限制

源码核对确认 terrain/radiation/time 分量及对应权重/开关仍进入 edge cost；本次结果重点是机制与回归，不替代 Gazebo 端到端验收。下一步需运行一次默认 adaptive=true 的短 Gazebo 回归，并检查实际 planner status/log 中的 sampling counters、Safety 唯一 `/cmd_vel` 与终点停车。

## 一次 Gazebo 回归（2026-08-22）

按 `/tmp` 启动 terrain、无 GUI Gazebo 和 online radiation planner，显式传入 `enable_adaptive_sampling:=true`；readiness 返回 `INPUTS_READY`、`GOAL_PUBLISHED: (5.700, 5.980)`、`PATH_READY: 3 poses`。`runtime/metrics.csv` 记录 `SUCCESS`，示例成功记录为 `path_points=3`、`path_length_m=0.340278`、`planning_time_sec=0.142286`、`iterations=1801`、`samples_uniform=1032`、`samples_goal=537`、`samples_risk=231`、`guided=1800`、`adaptations=17`，端到端 planner 规划和 adaptive 采样计数通过。

本次录包命令因预先创建了 rosbag 输出目录而被 rosbag 自身拒绝（`Output folder ... already exists`），故没有生成 bag，不能据此声称已完成倒车、终停或唯一 `/cmd_vel` 的消息级验收；没有重试 Gazebo。所有 launch 进程随后已清理。完整运行日志和 metrics CSV 保留在 `runtime/`，本次结论为“planner/readiness/采样计数通过，控制层消息级验收未完成”。

## 纠正夹具后的唯一尝试

新 bag 路径 `adaptive_true_valid_bag/` 未预创建，rosbag 成功打开并生成 `adaptive_true_valid_bag_0.db3`。但 planner launch 参数误写为 `metrics_csv=<path>` 而非 ROS 需要的 `metrics_csv:=<path>`，launch 立即退出并记录 `malformed launch argument`；因此 readiness 超时等待 risk map，未发布 goal/path，bag 不构成有效控制实验。该次已按要求停止且不重试；所有进程已清理。证据：`runtime/stack_valid.log`、`runtime/bag_valid.log`、`adaptive_true_valid_bag/`。最终 Gazebo 控制验收仍为未通过/未完成，不能宣称 GOAL_REACHED、无倒车、终停或 Safety 唯一 `/cmd_vel`。
