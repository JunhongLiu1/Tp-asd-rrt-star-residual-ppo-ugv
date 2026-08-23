# TP-ASD-RRT* + bounded Residual PPO 最终验证报告

日期：2026-08-22
平台：ROS 2 Foxy、Gazebo Classic 11、Husky A200、Python/C++

## 结论

系统的规划、在线地图、Pure Pursuit、PI、bounded residual adapter 和
Safety Gate 主链路已经集成。TP-ASD-RRT* 合成规划验收、PPO 确定性代理
A/B、隔离推理 worker 和单次短距离 Gazebo smoke 均有通过证据。

新增的 hard-terrain Gazebo 固定路径压力矩阵没有证明 PPO 的性能泛化：
零残差完成 1/6，PPO 完成 0/6；PPO 相对零残差的完成率差为 -16.7 个百分点。
两组均没有反向命令、越界停车或 worker fault latch。结论是“安全回退链有效，
Gazebo 压力性能未通过”，不是可部署或性能优越。

## 分阶段结果

| 阶段 | 规模 | 结果 | 判定 |
|---|---:|---|---|
| TP-ASD-RRT* 合成配对 | 3 地图 × 30 seeds | 两配置均 30/30；自适应节点数减少 14.1% / 7.2% / 7.6% | 机制通过 |
| PPO 确定性代理 A/B | 100 paired seeds | 两者 100% 到达；269.15 → 196.47 步，-62.41 → -28.02 | 代理验收通过 |
| PPO 隔离 worker | 5 次真实 checkpoint 请求 | 合约、有限有界动作、关闭清理通过 | 集成通过 |
| Gazebo 短距离 learned-policy smoke | 1 条短路线 | 到达、无反向、无 worker fault、`/cmd_vel` 归零 | smoke 通过 |
| Gazebo hard-terrain 压力矩阵 | 6 场景 × 2 策略 | zero 1/6，PPO 0/6；均无反向/越界/worker latch | 性能不通过 |
| 实体 Husky | 0 | 未测试 | 不可部署 |

TP-ASD 节点降幅按 `(legacy-adaptive)/legacy` 由原始均值计算：open
14.1%，risk-wall 7.2%，gradient 7.6%。wall 和 gradient 的平均总代价略升，
因此这里只声称采样机制减少节点规模，不声称所有地图上总代价更优。

## Gazebo 扩展矩阵

场景包括横向偏置、航向偏置、90°转弯、S 曲线、中距离复合路径和
急停—换路—恢复。每次使用同一 hard-terrain 世界、辐射插件、PI 参数、
75 秒时限和 Safety Gate 限制；路径、世界位姿和在线地图统一为 `map` 坐标。

| 场景 | Zero residual | PPO | 安全异常 |
|---|---:|---:|---:|
| 横向偏置 | 到达，67.7 s | 超时 | 0 |
| 航向偏置 | 超时 | 超时 | 0 |
| 90°转弯 | 超时 | 超时 | 0 |
| S 曲线 | 超时 | 超时 | 0 |
| 中距离复合路径 | 超时 | 超时 | 0 |
| 急停—换路—恢复 | 超时 | 超时 | 0 worker/越界/反向；zero 有 3 个急停采样仍非零，需复核时序 |

急停非零计数不应被忽略。它来自 50 Hz 采样与异步命令话题，可能包含急停切换
边沿，但在没有带 sequence id 的同步记录前不能证明是纯采样伪影。实体部署前
必须做带时间戳/序列号的 Safety Gate 急停延迟验收。

## PI 与控制结果的正确表述

已接受的单次直线 bag 中，PID-off 与 frozen-PI 都到达且无反向命令，并各有
20 个终点零命令。bag 总时长为 22.37 s 与 21.72 s；原项目描述中的
20.07 s 与 19.97 s 是从运动窗口提取的到达时间，不能与 bag 总时长混用。
最大横向误差为 0.000111 m 与 0.0000881 m，最大航向误差为 0.00694 rad 与
0.00606 rad。这是单次短直线 A/B，不代表复杂路径统计优势。

## 建议用于项目介绍的修订文本

- 搭建地形/辐射建模、在线风险地图、TP-ASD-RRT*、Pure Pursuit、PI、bounded
  Residual PPO 与 Safety Gate 组成的 ROS 2/Gazebo Husky 导航链路，并统一
  `map`、风险栅格、规划路径与机器人位姿坐标。
- 在 open、risk-wall、gradient 三类合成地图、seeds 31–60 的 90 组配对中，
  自适应与基线均为 30/30，平均节点数分别减少 14.1%、7.2%、7.6%；该结果
  验证采样机制，但不声称所有地图总代价更低。
- 50k bounded Residual PPO 在 100 组确定性代理配对中与零残差均 100% 到达，
  平均步数从 269.15 降至 196.47，平均回报从 -62.41 提升至 -28.02。
- learned-policy 已通过隔离 worker 和单次 Gazebo 短程 smoke；新增 6 场景
  hard-terrain 压力矩阵中 zero 为 1/6、PPO 为 0/6，说明 sim-to-sim 动力学
  泛化仍未通过，当前 checkpoint 仅限离线/仿真研究，不可部署到实体机器人。

## 下一步准入条件

1. 用 resettable Gazebo 环境或 rosbag transition 数据重新训练，而不是继续堆叠
   当前代理环境步数。
2. 给 baseline/output 增加同源 sequence id，准确验证每个策略残差和急停延迟。
3. 至少 3 个出生点、6 类场景、每策略每场景不少于 5 次配对；要求成功率不低于
   zero baseline、零安全违规，并报告置信区间。
4. Gazebo 通过后才能进入封闭场地实体 Husky 测试。

原始入口：`acceptance_logs/residual_gazebo_matrix_20260822/summary.csv`、
`summary.json`；重构后的跨阶段数据位于
`analysis_datasets/final_results_20260822/`。
