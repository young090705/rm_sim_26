# RM_SIM

## 项目简介

本项目是为了26赛季的视觉组算法仿真而创建，当前聚焦哨兵导航仿真：全向轮底盘（真车构型）与从 `rmoss_ws` 复制的 RMUC 2026 社区场地。机械臂仿真已移出范围（26赛季无工程机器人）。

哨兵模型 `models/sentry/` 的几何、质量、惯量与传感器位姿提取自 SolidWorks 总装图纸（烧饼备份11.14），非手工估算。

## 环境要求

- Ubuntu 24.04
- Gazebo Sim Harmonic
- ROS 2 Jazzy

## 雷达仿真模式

本项目支持两种雷达仿真方案，可通过配置文件 `config/sim_config.yaml` 或启动参数 `lidar_mode:=...` 切换：

### 1. RGL模式 (需要NVIDIA显卡)

使用 [RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin) 进行高精度 Livox Mid360 仿真。
默认采用 `Livox Mid360` 预设，800000 条射线分成 40 帧交替扫描，
每帧 20000 条射线、10 Hz（200000 条射线/秒），4 秒 pattern 循环。
仅命中场景的点会发布，故每帧有效回波数通常小于 20000。
默认几何量程为 0.1～40 m，与通用模式一致；这是固定量程裁剪，
不模拟实物随材质反射率变化的探测距离。
不要将整份文件直接配置为 `pattern_preset_path`：该用法会把 40 帧合成一帧。

```yaml
lidar_mode: "rgl"
```

**要求**：
- NVIDIA显卡 + 驱动
- 编译安装RGLGazeboPlugin

`rgl` 是严格模式：缺少驱动库、插件或 pattern 时启动报错，不会悄悄切换扫描模型。
只有 `auto` 模式会回退。安装步骤见下方「RGL 模式准备」。

当前插件输出 `x/y/z/intensity`，没有逐点时间、Livox CustomMsg、扫描运动畸变补偿或
真实材质反射率/噪声模型；`intensity` 来自场景 LaserRetro 设置。
其精确之处是预设射线方向和场景几何求交，不能宣称与实物所有误差特性一致。

### 2. gpu_lidar模式 (通用)

使用 Gazebo 内置的 `gpu_lidar`，无需 NVIDIA 显卡，但需要可用的 OpenGL/EGL 渲染环境。
它按 MID360 的 360° 水平、-7°～52° 垂直视场生成规则扫描点云，
**不复现 MID360 非重复扫描，不提供逐点时间或 Livox CustomMsg**。
点云为 `sensor_msgs/msg/PointCloud2`，不是 Livox 驱动的数据包。

```yaml
lidar_mode: "gpu_lidar"
```

### 3. 自动检测（可选）

检测 NVIDIA 库、RGL 插件目录和 pattern 文件；缺少所需资源时回退到 gpu_lidar。
当前机器已编译安装 RGL，配置默认为 `lidar_mode: "rgl"`。

```yaml
lidar_mode: "auto"
```

## 哨兵模型 (models/sentry)

从 SolidWorks 总装提取的实测参数：

| 项 | 值 | 来源 |
|---|---|---|
| 全向轮 | 直径 152 mm、宽 39 mm，4 轮 X 布局 | 网格包围盒 |
| 轮安装半径 | 245 mm（轮心距回转轴心） | 装配变换矩阵 |
| 底盘离地 | 96 mm（base_link 原点在轮心上方 20 mm） | 装配变换矩阵 |
| 整机质量 | 35.44 kg（底盘 13.56 + 云台 12.03 + 其余） | SW 质量属性 |
| 大 yaw | DM4310 + 同步带 + 滑环，无限位 | BOM |
| 小 yaw | GM6020 直驱，轴心前置 58.5 mm、高 121 mm | 装配变换矩阵 |
| pitch | DM4310 + 连杆，铰点在小 yaw 上方 169 mm | 轴承位姿 |
| Mid360 | 水平倒置，通过新增短悬臂**随大 yaw 回转** | 仿真改装，支架为简化估计件 |
| 自瞄相机/IMU | MV-CA016 与 DM-IMU-L1 均在 pitch 体上 | 装配变换矩阵 |

TF 树：`base_link → gimbal_link(大yaw) → small_yaw_link → pitch_link`，雷达安装链为 `gimbal_link → lidar_mount_link → livox_lidar`。

网格为 GLB 格式（Onshape 导出即 glTF，GLB 是其二进制封装），保留零件层级与材质。

外观由 `scripts/cad/recolor.py` 生成，两层规则：

1. **按名字覆盖**（优先）：裁判系统模块（装甲、灯条、测速、主控、场地交互、定位）为白色塑料，MV-CA016 相机为银灰铝，Livox Mid360 为深灰金属，全向轮辊子为白色聚氨酯，C 板为墨绿 PCB。
2. **按 CAD 材料密度分类**（其余结构件）：玻纤板与打印件哑光黑，铝件与钢件保持金属本色。

原始外观中高饱和度的零件（镜头、灯条）保留其色相。

改配色只需改 `scripts/cad/recolor.py` 的 `OVERRIDES` 与 `PALETTE_SRGB`（CAD 流水线脚本的数据依赖见 `scripts/cad/README.md`）。注意 glTF 的 `baseColorFactor` 是**线性空间**值，渲染时转 sRGB 显示，直接填 0.145 会显示成 RGB≈107 的灰色，脚本已做 sRGB→线性换算。

### 话题接口

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | Twist | 底盘速度，经 `omni_drive` 节点解算为四轮速度 |
| `/gimbal/big_yaw/cmd_pos`、`/gimbal/small_yaw/cmd_pos`、`/gimbal/pitch/cmd_pos` | Float64 | 云台三轴位置指令 |
| `/livox/lidar`、`/livox/imu` | PointCloud2、Imu | Mid360 |
| `/gimbal/imu` | Imu | 自瞄 IMU（pitch 体） |
| `/joint_states` | JointState | 7 个关节 |
| `/ground_truth/poses` | TFMessage | Gazebo 模型和连杆真值位姿，用于检查运动；不自动接入 `/tf` |

点云和 Livox IMU 的 `header.frame_id` 均为 `livox_lidar`，时间戳使用仿真时钟。
默认点云 10 Hz、Livox IMU 100 Hz（仿真时间），实际墙钟频率随实时率变化。
标准算法订阅点云/IMU 时建议使用 ROS 2 sensor-data QoS。

### 云台惯量与参数辨识

SDF 的 `<inertia>` 填的是**关于连杆质心、沿连杆坐标轴**的张量。辨识 `τ = J·θ̈ + b·θ̇ + f_c·sgn(θ̇)` 用的是**关于转轴**的标量 J，两者差一个平行轴项：

下表为 `scripts/check_inertia.py` 直接从 `model.sdf` 解析算出的值，即仿真里实际生效的量（非手算）：

| 轴 | J (kg·m²) | 下游质量 kg | 来源与可信度 |
|---|---|---|---|
| 大 yaw | **0.2316** | 13.093 | CAD 与外购件质量修正，加上新增短悬臂的估计惯量；雷达已抬高并倒置 |
| 小 yaw | **0.0185** | 3.446 | CAD 精确值。质心离轴 43.7 mm，平行轴项占 30.4% |
| pitch | **0.0149** | 1.200 | 估计值。CAD 无独立子装配 |
| 单轮 | 0.0032 | 1.343 | 绕自转轴 |

改动模型后重新核对：`python3 scripts/check_inertia.py models/sentry/model.sdf`

注意小 yaw 一项：GM6020 质量从 468 g 补到 960 g 后 J **完全不变**，因为电机正装在小 yaw 转轴上，到轴的垂直距离为零。这是个有用的自检——补质量若让该值变了，说明位置或坐标系算错了。

pitch 体的质量由物理约束定界：小 yaw 总成 2.954 kg 是 CAD 精确值，减去 pitch 估计后若出现负惯量即不可能，据此得 **m_pitch ≤ 1.65 kg**；模型取 1.2 kg。pitch 重力力矩峰值约 0.75 N·m（力臂 64.3 mm）。

#### CAD 材料核查（密度 = 质量 ÷ 体积 反推）

板材是对的，误差在外购件。结构板件在 CAD 中均为 ρ=2000 kg/m³，属玻纤/G10 范围，与实车黑色玻纤板一致（云台底板、弹仓板、悬挂板、防护件、装甲板固定、3508 安装板均为此值）。典型 G10 实测 1850~1950，CAD 取 2000 偏高约 3~8%，可忽略。

真正的误差源是**外购件未赋材料**——共 79 个零件密度≈1000 kg/m³，正是 SolidWorks 未设材料时的默认水密度：

| 零件 | CAD | 实物 | 比值 |
|---|---|---|---|
| M3508+减速箱 | 60.7 g | ~365 g | 0.17× |
| C620 电调 | 8.3 g | ~83 g | 0.10× |
| Livox Mid360 | 52.6 g | ~265 g | 0.20× |
| GM6020 | 468 g | ~960 g | 0.49× |
| MV-CA016 相机 | 110 g | ~55 g | 2.0× |

这些修正已应用到 `model.sdf`，每个 link 的注释标明了来源档次。

**整机质量不能直接用 CAD 根装配的 35.44 kg**——该装配含"干涉/范围"幻影件（做间隙检查用的非实体几何，`定位模块干涉范围` 单件就报 100.7 kg，`装甲板干涉` 报 376.8 kg），其中若干为隐藏件仍计入总质量。真实结构应取两个实体子装配之和：底盘 13.56 + 云台 12.03 = **25.59 kg**。原模型质量约 27.51 kg，差额为上表的外购件补足量；新增雷达支架约 0.357 kg（按实体铝件估算），改装后总质量约 27.87 kg。

实车称重是最终判据。

**用于辨识时必须注意：**

- CAD 惯量是**先验初值**，不是真值。实测 J 通常大于 CAD 值，因为电机转子惯量按传动比平方折算后计入负载：`J_eff = J_load + N²·J_rotor`。
- 大 yaw 的同步带经 CAD 实测为 **1:1**（主动轮与从动轮均为 63.51 mm），因此 N 只来自 DM4310 自身减速比，带传动不额外放大。
- CAD 完全不含摩擦、带弹性、线缆拖拽、齿隙，这些必须靠辨识获得。
- 原 CAD 连杆的惯量积近似写为 0；新增雷达支架保留按几何估计的惯量积（CAD 实际量级：云台 max|P|=0.033，约为最小主惯量的 15%；小 yaw 为 43%）。这会让惯量主轴与几何轴不重合，影响轴承反力与耦合项，但**不影响绕该轴的标量 J**，故对单轴辨识无影响。
- 质量值依赖 SolidWorks 材料设置，电机、NUC、电池等外购件模型密度常失真。整机 CAD 值 35.44 kg 应与实车称重核对。

建议流程：以上表 J 为初值做正弦扫频或阶跃辨识，若结果与 CAD 值相差在 1.5 倍以内属正常（转子折算 + 摩擦），相差数倍则应回头核查质量设置。

### 待实测校准的量

以下取自 CAD 或估算，实车测得后应替换：

- pitch 与小 yaw 的关节限位（当前为估计值）
- pitch 体质量与惯量（CAD 无独立子装配，按 1.0 kg 拆分）
- 云台三轴 PID 增益（当前为初值，云台参数辨识后校准）
- 各质量属性依赖 SW 材料设置，外购件模型常失真，建议整机称重校准
- Mid360 光学中心相对 CAD 装配原点的偏移（估计 <50 mm）
- `scripts/omni_drive.py` 的 `SIGN` 常量（若车运动方向相反则取反）

## 切换机器人模型

`config/sim_config.yaml`：

```yaml
robot_model: "sentry"       # 26赛季哨兵（默认）
# robot_model: "mecanum_car"  # 旧麦轮测试车
```

## 快速开始

```bash
cd /home/rm-young/rm_sim_26
source /opt/ros/jazzy/setup.bash

# 构建
colcon build --packages-select rm_sim_26

# 运行仿真
source install/setup.bash
ros2 launch rm_sim_26 rmuc_sim.launch.py
```

统一使用 `rmuc_sim.launch.py`，在 `config/sim_config.yaml` 中选择场地年份：

```yaml
map_year: 2026  # 改为 2025 即切换到 2025 场地
```

同一 YAML 的 `maps` 中配置各年份的 world 文件和出生点，默认选择 2026 社区场地。

无 GUI 运行（仍需要渲染点云）：

```bash
ros2 launch rm_sim_26 rmuc_sim.launch.py headless:=true lidar_mode:=gpu_lidar
```

场景 `worlds/RMUC2026_world.world` 使用从
`/home/rm-young/rmoss_ws/maps/rmuc2026/models` **完整复制**的 `rmuc_2026` 与 `speed_bump`。
场地 STL 约 6 MB、126101 个三角面，保留原模型 `0.1` 缩放、link 位姿与 world 位姿；
减速带保留 `0.001` 缩放及两处放置。运行不依赖原来的 `rmoss_ws`。
来源为 [git-alphabet/simulation](https://github.com/git-alphabet/simulation)
提交 `11561067834255e7b15bf0be89de2deaa15e9c07`，作者信息与来源说明保存在模型目录。
这是社区模型，尚未核验与官方最终场地的一致性。

2026 场地出生点为 `(-10, 0, 0.65) m`，该处地图上表面约为 `z=0.358656 m`；2025 场地沿用 `(3.4, 9.5, 0.3) m`。
可通过 `spawn_x:=... spawn_y:=... spawn_z:=...` 调整，需检查局部地形高度。
场景保留现有 GUI、物理、机器人生成、相机、雷达和 IMU 系统。
修改资源后重新构建，以同步到 ROS 包安装目录。

### 控制指令

在另一终端加载 ROS 和 `install/setup.bash` 后执行：

```bash
# 底盘坐标系：x 向前、y 向左，单位 m/s；angular.z 单位 rad/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}'

# 停止上述发布后，发零速度停车
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'

# 云台位置指令，单位 rad
ros2 topic pub --once /gimbal/big_yaw/cmd_pos std_msgs/msg/Float64 '{data: 0.3}'
```

Harmonic 使用 `gz_frame_id` 设置传感器消息坐标系。SDF 校验器可能对此扩展标签打印
“not defined in SDF”警告；Gazebo 仍会读取该设置。
参见 [Gazebo 官方 Harmonic 迁移说明](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/)。

## 仓库结构

- `config/`: 仿真模式与桥接配置
- `launch/`: Gazebo / ROS 2 启动文件
- `models/`: 机器人与场地模型资源
- `plugin/`: Livox Mid360 雷达pattern文件
- `worlds/`: Gazebo world文件

## RGL 模式准备

当前机器已在 RTX 4060 Laptop / NVIDIA 580.178.04 上完成编译安装。
安装目录为 `external/RGLGazeboPlugin/install`，不需要写入系统插件目录。
插件构建会下载官方 RGL 0.21.0 核心库，**无需另装 nvcc/CUDA Toolkit**。
从新检出重现安装（需要网络与 Gazebo Harmonic 开发依赖）：

```bash
cd /home/rm-young/rm_sim_26
source /opt/ros/jazzy/setup.bash
git submodule update --init --recursive external/RGLGazeboPlugin
cmake -S external/RGLGazeboPlugin -B external/RGLGazeboPlugin/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/external/RGLGazeboPlugin/install"
cmake --build external/RGLGazeboPlugin/build --parallel 2
cmake --install external/RGLGazeboPlugin/build
colcon build --packages-select rm_sim_26
source install/setup.bash
ros2 launch rm_sim_26 rmuc_sim.launch.py lidar_mode:=rgl
```

启动日志应显示 `雷达仿真模式: RGL (NVIDIA)`、插件路径、40 帧交替 pattern，
并在 Gazebo 日志中显示 `Running on GPU` 和 `Successfully loaded pattern`。
ROS 发布 `/livox/lidar`（PointCloud2，XYZI）与 `/livox/imu`（Imu）；
`/cmd_vel` 和云台控制接口保持不变。

插件路径自动从启动文件所在目录逐级查找，支持 source 与 install 布局。
另行安装时，可将 `RGL_GZ_PLUGIN_ROOT` 或 YAML `rgl_plugin_root` 设为工作区根、
RGL 源码目录或 RGL 的 install 目录。pattern 默认使用本包自带的官方
`plugin/LivoxMid360.mat3x4f`；也可通过 `RGL_PATTERNS_DIR` 指定该文件所在目录。

哨兵 CAD 原网格将 MID360 外壳并入云台，射线会先命中自身外壳并被近距过滤。
现将其精确拆为 `gimbal_link_structure.glb` 与 `mid360_housing.glb`，
外壳 visual 挂在 `livox_lidar` 下供 RGL 忽略，URDF 同样使用分离后的结构网格，避免出现两个雷达。
雷达当前位于大 yaw 坐标系 `(0, 0, 0.5000) m`，即大 yaw 轴线正上方；短悬臂约 216 mm，roll 为 π，保持原 yaw：
传感器 +Z 朝下，外壳、扫描、IMU 与 TF 一起翻转。新增短悬臂包含可见几何、碰撞体和估计惯量，
其遮挡会参与射线求交；不能保证完全没有车体遮挡。RViz 应使用 `base_link` 查看，
若使用 `livox_lidar` 作 Fixed Frame，倒置坐标系会使地面看起来翻转。
重新导出云台 CAD 后运行 `python3 scripts/cad/split_mid360_visual.py`，再构建。

构建方式与预设说明参考 [RGLGazeboPlugin 官方文档](https://github.com/RobotecAI/RGLGazeboPlugin)。

## TODO

- ✅ 给机器人加入云台结构
- ✅ 机器人能够发布TF
- ✅ 加入比赛场地模型
- ✅ 实现Mid360的仿真
- ✅ 支持无NVIDIA显卡的仿真方案
- ✅ 车身模型对齐真车：全向轮底盘 + 三轴云台，几何取自 CAD 总装
- [ ] 修正每次启动的视角
- [ ] 用实测值替换上文「待实测校准的量」
- [ ] 碰撞体细化（当前底盘/云台为圆柱包络，轮为球）

## 致谢

致敬[RGLGazeboPlugin](https://github.com/RobotecAI/RGLGazeboPlugin)开发者的贡献
