# TP-ASD-RRT* + Residual PPO 风险感知 UGV

本仓库是 ROS 2 Foxy 下的最终交付版本，只包含当前风险感知移动机器人系统。主链路为：地形/辐射感知 → 在线风险地图 → 路径规划 → 路径跟踪与 PID → Residual PPO 修正 → Safety Gate → `/cmd_vel`。

## 正式算法名称

| 层级 | 算法名称 | 作用 | 主要实现 |
| --- | --- | --- | --- |
| 全局规划 | **TP-ASD-RRT\***（Time-Penalized Adaptive Sampling Distribution RRT*，时间惩罚自适应采样分布 RRT*） | 联合地形与辐射风险，以 uniform/goal/risk 混合采样、目标吸引和风险负梯度 APF 完成在线规划 | `src/risk_aware_planner_cpp/src/planner_core.cpp` |
| 风险代价 | **Terrain–Radiation–Time Edge Cost**（地形–辐射–时间边代价） | 对候选边的长度、地形阻抗、辐射剂量与时间惩罚进行加权 | `src/risk_aware_planner_cpp/include/risk_aware_planner_cpp/edge_cost_model.hpp` |
| 路径跟踪 | **Pure Pursuit + PID**（纯追踪 + 比例积分微分控制） | 生成基准线速度和角速度，包含限幅与 anti-windup | `src/risk_aware_planner_cpp/src/pure_pursuit_core.cpp`、`pid_core.cpp` |
| 残差控制 | **Safety-Bounded Residual PPO**（安全边界残差近端策略优化） | 在 PID 基准命令上施加有界残差，不允许倒车 | `src/risk_aware_residual_rl/risk_aware_residual_rl/` |
| 最终安全控制 | **Fail-Closed Safety Gate**（故障闭锁安全门） | 检查输入新鲜度、有限性、状态、急停和风险地图版本，并独占最终 `/cmd_vel` | `src/risk_aware_planner_cpp/src/cmd_vel_safety_node.cpp` |
| 在线建图 | **Terrain–Radiation Risk Mapping**（地形–辐射风险融合建图） | 将车辆感知地形和辐射测量转换为规划器风险地图 | `src/risk_aware_planner_cpp/src/radiation_online_mapper_node.cpp`、`src/radiation_mapping/` |

其中唯一正式全局规划算法名为 **TP-ASD-RRT\***。配置 `enable_adaptive_sampling=false` 仅用于消融测试，对应 **Legacy Risk-Weighted RRT\* Baseline**，不是另一套交付算法。

## 仓库结构

- `src/risk_aware_planner_cpp/`：TP-ASD-RRT*、风险代价、Pure Pursuit、PID、Safety Gate、启动文件与单元测试。
- `src/risk_aware_residual_rl/`：Residual PPO 环境、训练/评估、推理 worker、安全适配器与测试。
- `src/radiation_mapping/`：正式地形/辐射节点、Gazebo 启动文件、配置和最终 world。
- `src/gazebo_radiation_plugins/`：Gazebo 辐射场插件。
- `src/radiation_interfaces/`：系统使用的 ROS 消息、服务和 action 接口。

## 构建与测试

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

规划器启动参数和运行方式见 `src/risk_aware_planner_cpp/README.md`；Residual PPO 的训练、评估和安全契约见 `src/risk_aware_residual_rl/README.md`。
