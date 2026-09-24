# 调参修改记录

## 1. 修复 demo 视频无法追踪 (2026-06-21)

### 1.1 颜色过滤 — `configs/standard3.yaml`
```yaml
# 修改前
enemy_color: "red"

# 修改后
# enemy_color: "red"
enemy_color: "blue"
```
**原因**：demo 视频中的装甲板是蓝色，配置却是红色。Tracker 的 `armors.remove_if(color != enemy_color_)` 把检测到的蓝色装甲板全部过滤掉了，导致 tracker 永远返回空列表，aimer 无输出。

### 1.2 瞄准角度放宽 — `configs/standard3.yaml`
```yaml
# 修改前
comming_angle: 55
leaving_angle: 20

# 修改后
comming_angle: 70
leaving_angle: 5
```
**原因**：demo 视频中目标在旋转，装甲板角度变化大。原配置 55°/20° 太严格，choose_aim_point 遍历所有装甲板后没有一块满足条件，返回 invalid。放宽进入角、收窄离开角后红框出现。

---

## 2. 新增离线 MPC 测试程序 (2026-06-21)

### 2.1 新建 `tests/auto_aim_mpc_test.cpp`
`auto_aim_test` 用的是传统 Aimer，新文件替换为 Planner (MPC)，同时输出 PlotJuggler UDP 数据。用于离线测试 MPC 轨迹规划效果。

### 2.2 修改 `CMakeLists.txt`
- 添加 `add_executable(auto_aim_mpc_test tests/auto_aim_mpc_test.cpp)`
- 添加 `target_link_libraries(auto_aim_mpc_test ...)`

### 2.3 修复 Planner 瞄准点颜色 (`tests/auto_aim_mpc_test.cpp`)
```cpp
// 修改前 (BGR 下是蓝色)
tools::draw_points(img, aim_pts, {255, 0, 0});

// 修改后 (红色)
tools::draw_points(img, aim_pts, {0, 0, 255});
```

### 2.4 Planner 诊断日志 (`tasks/auto_aim/planner/planner.cpp`)
```cpp
// 修改前：仅输出 "Unsolvable target"
// 修改后：输出具体数值
tools::logger()->warn("[Planner] Unsolvable: d={:.2f} z={:.2f} spd={:.2f} err={}",
                      min_dist, xyz.z(), bullet_speed, e.what());
```
**原因**：MPC 离线测试时频繁返回 control=false，需要知道是弹道不可解还是其他原因。日志显示 EKF 发散导致 d=17m z=24m 的离谱值。

### 2.5 测试诊断日志 (`tests/auto_aim_test.cpp`)
添加了每帧 Z 坐标对比日志（调试用，待清理）：
```cpp
tools::logger()->warn("Z: ekf={:.3f} vz={:.3f} aim={:.3f} diff={:.3f}", ...);
```
**原因**：诊断红框上下预测方向是否与 EKF vz 符号一致。

---

## 3. 测试结论

| 程序 | 适用场景 | 当前效果 |
|------|---------|---------|
| `auto_aim_test` (传统 Aimer) | 简单平移、低速旋转 | 红框稳定，可调 yaw/pitch offset |
| `auto_aim_mpc_test` (MPC Planner) | 高速小陀螺（装甲板切换） | 离线效果差 — EKF vz 振荡导致弹道无解 |

**对于手持上下移动靶场景，推荐使用 `auto_aim_test`。** MPC 的设计目标是"装甲板切换时的轨迹平滑"，不适用于简单平移。

---

## 4. Z轴跟随速度优化 (2026-06-21)

### 4.1 拆分 Z 轴过程噪声 — `tasks/auto_aim/target.cpp`
```cpp
// 修改前：x/y/z 三轴共用 v1=100
v1 = 100;
// Q矩阵 z行: a*v1, b*v1, ...

// 修改后：Z轴独立使用 v1_z=400
v1 = 100;    // x, y 轴保持原值
v1_z = 400;  // Z轴提高4倍
// Q矩阵 z行: a*v1_z, b*v1_z, ...
```
**原因**：EKF 的 Z 轴过程噪声（Q矩阵）决定了滤波器对上下方向速度变化的响应速度。原值 100（σ_a=10m/s²）对手持快速上下移动的场景太保守，vz 收敛慢导致瞄准点滞后。提高到 400（σ_a=20m/s²）让 EKF 更愿意接受较大的 Z 方向速度变化，红框跟随速度更快。

**副作用**：过程噪声增大会导致 vz 估计的短时波动增大，但因为场景是简单上下平移（无旋转装甲板切换），影响可接受。

### 4.2 降低距离观测噪声 — `tasks/auto_aim/target.cpp`
```cpp
// 修改前：距离观测噪声 ≈ log(|da|+1)+1 ≈ 1.0 (std≈1m)
Eigen::VectorXd R_dig{{4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1, ...}};

// 修改后：距离观测噪声 ≈ 0.15 (std≈0.39m)
Eigen::VectorXd R_dig{{4e-3, 4e-3, log(std::abs(delta_angle) + 1) * 0.15 + 0.05, ...}};
```
**原因**：这是pitch滞后的根因。EKF观测是YPD球坐标(yaw, pitch, distance, angle)。距离噪声1m²意味着EKF对PnP深度观测极不信任 → Kalman增益很小 → z和vz更新缓慢。PnP在近距(1-2m)的深度精度远好于1m，降低此噪声让EKF更信任新观测，vz收敛速度提升约6倍。

### 4.3 Pitch滞后原因详解

Yaw不滞后是因为EKF有**直接状态**x[6]=yaw和x[7]=vyaw, 观测也直接包含yaw。
Pitch滞后是因为**没有直接的pitch状态** — EKF要通过pitch+distance两个间接观测，
经Jacobian映射到x[4]=z, 再从Δz/Δt反推x[5]=vz。距离噪声大→Kalman增益小→从观测到vz的信息流被压制。

---

## 5. 待清理的临时诊断代码

以下文件包含临时诊断日志，调参完成后可清理：

- `tests/auto_aim_test.cpp` — Z 坐标对比日志（约第162行）
- `tasks/auto_aim/planner/planner.cpp` — Unsolvable 详细日志（约第54行）

运行方式：
```bash
# 清理测试
grep -n "warn.*Z:" tests/auto_aim_test.cpp
grep -n "warn.*Unsolvable" tasks/auto_aim/planner/planner.cpp
```
