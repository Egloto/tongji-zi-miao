# SP Vision 25 自瞄笔记

## 编译

```bash
cmake -B build
make -C build/ -j`nproc`
```

依赖：OpenCV、Eigen、OpenVINO、Ceres、spdlog、yaml-cpp、fmt 等（详见 readme.md）

---

## 运行

### 离线测试（录播视频回放）

```bash
./build/auto_aim_test
```

参数：
- `-c` / `--config-path`：yaml 配置（默认 `configs/demo.yaml`）
- `-s` / `--start-index`：起始帧（默认 0）
- `-e` / `--end-index`：结束帧（默认 0 = 到末尾）
- `@input-path`：视频路径，读 `{path}.avi` + `{path}.txt`（默认 `assets/demo/demo`）

### 实车运行

```bash
./build/standard_mpc configs/standard3.yaml   # 步兵（MPC规划器）
./build/standard_mpc configs/sentry.yaml       # 哨兵
./build/standard_mpc configs/uav.yaml          # 无人机
./build/standard configs/standard3.yaml        # 步兵（传统分段决策器）
```

---

## 自瞄流程（四步）

| 步骤 | 模块 | 作用 |
|------|------|------|
| 1. 识别 | YOLO (`yolo.detect`) | 神经网络检测装甲板四点坐标 |
| 2. 位姿解算 | Solver (`solver.set_R_gimbal2world`) | 云台姿态 → 世界坐标系转换 |
| 3. 追踪 | Tracker (`tracker.track`) | 扩展卡尔曼滤波（EKF）估计目标运动状态 |
| 4. 决策 | Aimer (`aimer.aim`) | 轨迹规划 + 瞄准位置 + 开火判断 |

---

## MPC（模型预测控制）

### 解决的问题

小陀螺旋转时装甲板切换导致目标轨迹突变，云台跟不上。

### 原理

用 TinyMPC 求解二次规划：
- **变量**：规划后轨迹的加速度序列
- **目标**：尽量贴合射击轨迹（提高命中率）
- **约束**：加速度 ≤ 云台最大加速度

### 效果

| | 传统分段决策 | MPC 规划器 |
|---|---|---|
| 决策方式 | 经验分段逻辑，多套参数 | 数学优化，统一求解 |
| 调参 | 手动调多组参数 | 只需物理参数（云台最大加速度等） |
| 击杀时间 | - | ~10s（300HP，14rad/s，2m） |

---

## 模式切换

电控通过下位机发送模式信号，程序根据模式执行对应逻辑：

- `AUTO_AIM` → 自瞄（装甲板检测 + 追踪 + 射击）
- `SMALL_BUFF` → 打小符（能量机关）
- `BIG_BUFF` → 大打符
- `IDLE` → 空闲，不发送控制指令

---

## 可视化调试

- **PlotJuggler**：连接 UDP 9870 端口，实时查看轨迹、EKF 状态、开火信号等曲线
- **窗口显示**：绿色框 = 检测装甲板，红色点 = 瞄准位置，按 `q` 退出
