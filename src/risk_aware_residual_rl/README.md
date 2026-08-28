# 安全边界残差 PPO 基础设施

本软件包是地形/辐射导航栈中首个可测试的强化学习层。它**不会**替换、调优或绕过 Pure Pursuit、冻结的 PI 控制器或确定性安全门。仓库中的 1024 步代理环境 PPO 冒烟检查点仅用于证明流水线可运行，其标签为 **SMOKE_ONLY_NOT_DEPLOYABLE**，尚未获准用于 Gazebo、实体机器人、安全或性能部署。

## 控制契约

预期的命令链路如下：

```text
Pure Pursuit 参考值 -> 冻结的 PI 反馈 -> 基准命令
                                      + 有界策略残差
                                      -> /control/base_cmd
                                      -> 现有安全门
                                      -> /cmd_vel
```

策略 action 严格包含两个位于 `[-1, 1]` 的归一化值：

1. 线速度残差，最大缩放至 `0.02 m/s`；
2. 角速度残差，最大缩放至 `0.10 rad/s`。

进入安全门前的合成命令还会被限制为不可倒车的 `[0.0, 0.20] m/s` 和 `[-0.60, 0.60] rad/s`。这些限制属于纵深防御，不能替代安全门。零 action 会返回完全相同的基准 `Command` 对象；action 格式错误、非有限或推理失败时，系统会回退到未修改的有限基准值。若基准值本身非有限，残差层输出零，下游安全门必须始终启用。若有限基准值已超出残差层的绝对包络，则不应用学习修正，而是将原命令交给安全门，避免绝对限幅伪装成大幅策略残差。

## 观测

`observation.py` 定义了固定的 14 维归一化向量：

1. 基准和实际线速度；
2. 基准和实际角速度；
3. 线速度和角速度跟踪误差；
4. 横向误差和航向误差；
5. Pure Pursuit 曲率和剩余目标距离；
6. 辐射剂量率和地形阻抗；
7. 基准饱和标志和安全停车标志。

所有连续值都必须是有限值。无效观测会编码为带无效标志的全零向量，`SafeResidualController` 不会调用或信任策略输出，而是保留基准命令。

## 奖励与回合状态

`reward.py` 明确定义了奖励契约：向目标前进会获得奖励；横向/航向/速度误差、残差幅度及变化、辐射、地形阻抗和饱和都会受到惩罚；抵达目标会终止回合并获得奖励。碰撞、越界、安全门停车、无效 action 或非有限状态转移会以占主导的安全惩罚终止回合。达到时间上限属于 Gymnasium truncation，并不代表成功。

奖励权重只是基础设施测试的初始工程值。在进行长时间训练或宣称性能对比之前，必须结合已记录的 PID A/B 分布重新审查。

## 环境与可选依赖

`ResidualControlCoreEnv` 是确定性的一阶代理环境，不依赖 ROS、Gym、Torch 或 SB3，用于测试契约、随机种子和短 PPO 冒烟训练，不是经过验证的 Husky/Gazebo 动力学模型。

安装 Gymnasium 后，`make_gym_env()` 可提供 Gymnasium 包装器。PPO 训练和检查点加载还需要兼容版本的 Stable-Baselines3 与 Torch。这些均为延迟导入的可选依赖，ROS 软件包不会自动下载。Python 3.8 CPU 冒烟环境完整锁定在 `requirements-ppo-py38.txt`，其中也声明了 PyTorch CPU wheel 索引。工作区本地的 `.rl_deps/` 并非软件包数据，不应复制进 ROS 安装目录。

Ubuntu 系统的 `mpl_toolkits` 命名空间可能与通过 `--target` 安装的 Matplotlib 冲突。仅把 `.rl_deps/` 放到 `PYTHONPATH` 前面并不会排除 `/usr/lib/python3/dist-packages`。请使用不含系统 site-packages 的干净虚拟环境，或使用下述 `python3 -S` 方式：

```bash
cd /home/i/terrain_radiation_ws
python3 -m pip install \
  --target /home/i/terrain_radiation_ws/.rl_deps \
  --requirement src/risk_aware_residual_rl/requirements-ppo-py38.txt
```

安装依赖是明确的环境准备步骤，训练和评估命令不会自行安装依赖。不安装可选依赖也可以检查配置：

```bash
source /opt/ros/foxy/setup.bash
source /home/i/terrain_radiation_ws/install/setup.bash
ros2 run risk_aware_residual_rl residual_ppo_train \
  --config /home/i/terrain_radiation_ws/src/risk_aware_residual_rl/config/residual_ppo_default.json \
  --dry-run
```

准备好依赖后，可通过隔离的导入路径运行无 ROS 模块。以下命令复现 1024 步冒烟配置，但把结果写入 `/tmp`，不会修改证据目录：

```bash
cd /home/i/terrain_radiation_ws
PYTHONPATH=/home/i/terrain_radiation_ws/.rl_deps:/home/i/terrain_radiation_ws/src/risk_aware_residual_rl \
python3 -S -m risk_aware_residual_rl.training \
  --config /home/i/terrain_radiation_ws/src/risk_aware_residual_rl/config/residual_ppo_default.json \
  --seed 31 \
  --total-timesteps 1024 \
  --checkpoint-freq 512 \
  --checkpoint-dir /tmp/residual_ppo_reproduction \
  --device cpu
```

CLI 会写入周期性 SB3 检查点、最终检查点及准确的 JSON 训练配置，但不会将检查点标记为安全或进行部署。

已完成的流水线冒烟证据位于 `acceptance_logs/rl_ppo_20260822/smoke_checkpoints/`，包含 512 步、1024 步和最终检查点，以及 `training_config.json`。这些文件只能证明锁定的 CPU 训练流水线完成了 1024 个代理环境步骤，所有检查点均为 **SMOKE_ONLY_NOT_DEPLOYABLE**。

`acceptance_logs/rl_ppo_20260822/offline_evaluation_smoke.json` 记录了最终检查点在种子 31–33 上的三回合确定性 CLI 冒烟测试：900 个 action 均为有限且有界值，但三个回合都在 300 步时被截断，未到达目标。负回报和零成功终止只是流水线证据，不是正向性能结果；JSON 明确保持 `deployable: false`。

首次冒烟暴露了环境可达性缺陷：旧代理环境在 300 步、0.1 秒时间范围内采样 3–6 m 的目标，而 0.08 m/s 的标称基准速度在考虑地形和一阶滞后之前最多只能行驶 2.4 m；临近目标时速度还会趋近于零，导致无法触发终止。修正后的默认值为 0.8–1.8 m、400 步，并采用真实跟踪器 0.04 m/s 的最小跟踪速度。带种子的零残差和速度/航向启发式回归测试证明默认环境可达，且启发式策略用时更短、回报更高。旧的 1024 步检查点基于有缺陷的环境训练，不得继续训练、等价比较或部署。

训练会同时生成 `training_config.json` 和 `training_manifest.json`；后者记录环境、观测、action 限制和奖励契约，并始终标注为不可部署。不得手工为已有检查点添加契约字段，应使用真实的锁定 ML 解释器进行审计和原子化定稿：

```bash
PYTHONPATH=/home/i/terrain_radiation_ws/.rl_deps:/home/i/terrain_radiation_ws/src/risk_aware_residual_rl \
/usr/bin/python3 -S -m risk_aware_residual_rl.artifact_finalize \
  --checkpoint acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/residual_ppo_final.zip \
  --training-config acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/training_config.json \
  --manifest acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/training_manifest.json \
  --evidence acceptance_logs/rl_ppo_20260822/retrain50k_artifact_finalization.json
```

定稿器会实际加载检查点，而不是信任文件名；它要求保存的观测/action 空间与当前 Gym 环境完全一致，对照训练配置验证种子、按 rollout 取整的步数和 PPO 超参数，对两个输入计算哈希，生成当前契约，并在输出审计证据前运行 `validate_artifact_contract`。任何不匹配都会拒绝定稿，不允许通过编造元数据或重贴标签来修复。

已完成的 `config/residual_ppo_budget_cpu.json` 20k 训练未通过验收。在保留种子 1000–1099 上，零残差成功率为 100%、平均长度 260.57、平均回报 -58.15；PPO 成功率为 99%、长度 250.48、回报 -75.89。奖励分解显示损失几乎全部来自更差的航向、横向和速度跟踪；残差幅度/变化可忽略，且没有饱和或安全惩罚。诊断还显示确定性策略每一步都输出负角度 action，仅 21.9% 的时间与航向误差方向相反。

根因是角度策略塌缩且输入被过度压缩：航向使用 π rad、目标距离使用 10 m、辐射使用 8 单位作为缩放，而代理环境实际范围小得多。修正后的观测尺度分别为 1 rad、2 m、0.5 辐射单位和 50 地形单位。左右镜像测试用于防止动力学和奖励出现方向偏差，未移除任何安全、航向、横向或残差代价。

基于证据的重训候选配置为 `config/residual_ppo_retrain_cpu.json`：50k CPU 步、种子 47、1024 步 rollout，以及 0.001 的熵系数，以降低再次发生早期恒定 action 塌缩的风险。由于观测契约已过期，不得续训 20k 检查点。全新训练后，应在至少 100 个连续保留种子上评估，并同时满足：

- `goal_reached` 不低于 95%，时间上限截断不高于 5%；
- 无无效、非有限或越界 action，且安全终止次数为零；
- 在相同种子上，成功率不得比零残差基线低超过 2 个百分点；
- 平均回合长度更短，平均回报高于零残差基线。

任一门槛失败，检查点都必须保持仅离线使用。通过代理环境门槛也不代表获准用于 Gazebo 或实体机器人。50k 候选仅从头训练一次；若相同种子的回报或成功率仍不达标，或再次塌缩为单一符号的角度策略，应停止扩展 PPO 步数，转而研究对称感知采样/课程学习或策略架构。

使用确定性预测（默认）、连续回合种子和 JSON 输出离线评估指定检查点：

```bash
cd /home/i/terrain_radiation_ws
PYTHONPATH=/home/i/terrain_radiation_ws/.rl_deps:/home/i/terrain_radiation_ws/src/risk_aware_residual_rl \
python3 -S -m risk_aware_residual_rl.evaluation \
  --checkpoint acceptance_logs/rl_ppo_20260822/smoke_checkpoints/residual_ppo_final.zip \
  --episodes 10 \
  --seed 31 \
  --output /tmp/residual_ppo_offline_evaluation.json
```

`residual_ppo_evaluate` 也可在干净虚拟环境中作为控制台入口运行。默认情况下，它会在相同种子上分别运行检查点策略和零残差策略，聚合带符号的奖励分量，并直接应用上述门槛。JSON 会区分 `execution_valid` 与 `acceptance_passed`；评估执行有效时，验收仍可能失败。无效执行返回状态码 2，有效但被拒绝的 A/B 测试返回 3，通过代理 A/B 验收返回 0。`--no-zero-baseline` 只执行运行检查，并将 `acceptance_passed` 保持为 null。每次策略推理之前都会拒绝非有限、格式错误或越界的 14 维观测；环境 step 之前也会拒绝非有限、格式错误或超出 `[-1, 1]` 的 action。`--stochastic` 仅用于明确的诊断运行。即使验收成功，仍会设置 `deployable: false` 并保留 **SMOKE_ONLY_NOT_DEPLOYABLE** 标签。

## ROS 适配器与启动契约

`residual_policy_node` 实现了适配器。主实验和在线辐射启动文件公开 `enable_residual_rl`、策略/检查点与隔离 worker 选项、超时/退避参数，以及两个残差边界。

默认 `enable_residual_rl:=false` 时不会启动适配器，跟踪器仍按历史行为向 `/control/base_cmd` 输出。设为 true 时，启动文件会以原子方式将跟踪器重映射到私有的 `/control/pid_baseline_cmd`，并只启动一个向 `/control/base_cmd` 发布的残差节点。现有安全门继续订阅该话题，且仍是唯一向最终 `/cmd_vel` 发送命令的组件。

适配器启动后，在收到基准值前输出零。基准值缺失、过期或非有限时输出零；指标过期/缺失、模型超时、模型异常或策略 action 无效时回退至新鲜基准值；辅助指标/剂量非有限、跟踪器处于停止/到达目标/等待状态或急停时会清除状态并输出零。`enable_rl` 参数与 `/control/residual_rl_enable` Bool 话题是相互独立的运行时关闭开关；关闭时，新鲜基准值原样通过。

显式插入安全零策略进行冒烟测试：

```bash
ros2 launch risk_aware_planner_cpp \
  tp_asd_rrt_star_online_radiation.launch.py \
  enable_motion:=true \
  enable_residual_rl:=true \
  residual_policy_type:=zero
```

SB3 策略绝不会随机初始化。`residual_policy_type:=sb3` 必须提供明确的检查点、manifest 和 SHA-256 白名单项。若 artifact 缺失、哈希不匹配，或 manifest 中有序的 14 字段观测/缩放及归一化双 action 契约与运行中的适配器不同，启动会被拒绝。零策略不会创建推理 worker。

学习策略推理时，ROS 父进程不会导入 Torch 或 SB3，而是使用指定解释器、`-S` 和 `residual_worker_pythonpath` 启动单独的进程组。JSON Lines 请求包含单调递增的请求 ID、14 个有界观测值和严格两个有界 action。模型加载和启动握手会在控制回调开始前完成。典型的隔离 50k 候选命令如下：

```bash
ros2 launch risk_aware_planner_cpp \
  tp_asd_rrt_star_online_radiation.launch.py \
  enable_residual_rl:=true \
  residual_policy_type:=sb3 \
  residual_checkpoint_path:=/home/i/terrain_radiation_ws/acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/residual_ppo_final.zip \
  residual_checkpoint_manifest_path:=/home/i/terrain_radiation_ws/acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/training_manifest.json \
  residual_checkpoint_sha256_allowlist:=8233e2504909a97844cb3f97c72ab7c7756b1762a997a48168904256c2f1c742 \
  residual_worker_python_executable:=/usr/bin/python3 \
  residual_worker_pythonpath:=/home/i/terrain_radiation_ws/.rl_deps \
  residual_model_timeout_sec:=0.05
```

每次预测都有严格的 IPC 截止时间。worker 超时/崩溃、JSON 格式错误、请求 ID 错误、action 非有限或超出 `[-1, 1]` 时，系统会终止整个 worker 进程组，本周期使用新鲜 PID 基准值，并将 RL 故障锁定为禁用。控制期间不会自动恢复；诊断后，操作员必须显式地先关闭再开启 `/control/residual_rl_enable`，或依次将 `enable_rl` 设为 false、true。有限指数重启退避默认为 0.5–5 秒。急停也会终止并锁定 worker；节点关闭时会清理进程组，避免遗留孤儿进程。下游安全门始终拥有最终命令权。

零策略 Gazebo 接线冒烟证据保存在 `acceptance_logs/pid_rl_adapter_20260822/`。`runtime/readiness.log` 记录地图/里程计就绪及非空路径，`runtime/stack.log` 记录私有 PID 话题和零策略适配器启动，`zero_policy_bag/` 记录基准、残差输出、安全门输出、控制指标、路径和里程计话题。这些证据只验证适配器接线与零策略运行，不代表学习策略通过安全或性能审批。

`acceptance_logs/rl_ppo_20260822/retrain50k_worker_smoke.json` 记录了真实检查点经隔离 worker 完成的五次请求、契约验证、有限有界 action、父进程未导入 Torch/SB3，以及无孤儿进程的干净关闭。它属于非 ROS worker 冒烟测试，不是 Gazebo 审批。

后续训练应从代理环境转向记录的离线状态转移和可重置 Gazebo 适配器。部署前必须在未见种子上独立评估、通过全部现有急停测试，并显式加入检查点白名单。看门狗和离线代理环境门槛不能代替这些审批。
