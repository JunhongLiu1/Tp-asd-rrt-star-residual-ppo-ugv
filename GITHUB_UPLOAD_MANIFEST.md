# GitHub 上传清单

## 必须上传

- `src/risk_aware_residual_rl/`：bounded PPO、隔离 worker、评估器、测试和文档。
- `src/risk_aware_planner_cpp/`：TP-ASD-RRT*、follower、PI/PID、Safety Gate 和 launch。
- `src/radiation_mapping/`、`src/radiation_interfaces/`、`gazebo_radiation_plugins/`：
  地形/辐射数据链和 Gazebo 插件源码。
- `tools/pi_gazebo_trial.py`
- `tools/run_residual_gazebo_matrix.sh`
- `tools/analyze_residual_gazebo_matrix.py`
- `tools/export_final_results_dataset.py`
- `analysis_datasets/final_results_20260822/`
- `acceptance_logs/tp_asd_20260822/synthetic_ab_30seeds.json`
- `acceptance_logs/rl_ppo_20260822/retrain50k_ab_evaluation_100seeds.json`
- `acceptance_logs/rl_ppo_20260822/retrain50k_artifact_finalization.json`
- `acceptance_logs/rl_ppo_20260822/retrain50k_worker_smoke.json`
- `acceptance_logs/residual_gazebo_matrix_20260822/summary.csv`
- `acceptance_logs/residual_gazebo_matrix_20260822/summary.json`
- `acceptance_logs/pid_supplemental_20260822/report.json`
- `FINAL_VALIDATION_REPORT.md`、本清单和根目录 `README.md`。

## 建议上传或放 GitHub Release/LFS

- `retrain50k_checkpoints/residual_ppo_final.zip` 及其 `training_config.json`、
  `training_manifest.json`。checkpoint 应与 SHA-256 一起发布；若仓库限制大小，
  使用 Git LFS 或 GitHub Release，不要只上传 zip 而遗漏 manifest。
- 扩展矩阵每次运行的 `result.json` 和适量 `samples.csv`。完整 rosbag、Gazebo log
  和大 CSV 更适合 Release/LFS/对象存储，并在仓库保留哈希清单。
- `learned_gazebo_smoke_valid/learned_policy_bag`，用于第三方复核短程 smoke。

## 不要上传

- `.rl_deps/`、`build/`、`install/`、`log/`、`.pytest_cache/`、`__pycache__/`。
- `*.db3-shm`、`*.db3-wal`、运行中的 PID 文件、临时 `/tmp` 输出。
- `radiation_mapping` 根目录空文件；它会干扰 Foxy 的 `ros2 launch` 包名解析。
- 无来源说明的失败/调试日志和旧 checkpoint；需要保留时放独立归档，不要混入
  accepted evidence。

上传前先执行 `git status --short`，逐项确认未提交文件归属；本工作区原本已有
用户的未提交/未跟踪改动，不能用 `git add .` 无差别加入。
