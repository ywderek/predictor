# AGENTS.md

本文件面向 AI 编码代理，介绍本项目的架构、构建方式与开发约定。假定读者对本项目一无所知。

## 项目概述

这是一个 **ROS 2 (Kilted) colcon 工作空间**，用于 RoboMaster 装甲板视觉追踪。工作空间根目录下有 `src/`、`build/`、`install/`、`log/` 四个标准 colcon 目录。根目录本身不是 git 仓库。

包含两个包：

- **src/armor_tracker/**（C++，ament_cmake）：装甲板多目标追踪器。订阅 `/vision` 话题上的 `rm_msgs/DetectionArray` 检测消息，逐块装甲板做 PnP 解算（OpenCV `SOLVEPNP_IPPE`，小装甲板尺寸 0.135×0.054 m），经 tf2 从 `camera_optical_frame` 变换到 `gimbal_link` 坐标系，然后做最近邻帧间匹配 + 恒速模型卡尔曼滤波，并用摩尔投票（score 加权）确定每个目标的身份码（color×10+type）。结果以 `visualization_msgs/MarkerArray` 球体发布到 `/armor_markers` 供 rviz2 显示（绿色=观测、红色=0.1s 预测、黄色=本帧卡尔曼滤波结果，与绿色观测基本重合；目标本帧未匹配到检测时不发布该目标的任何球）。
- **src/rm-msgs/**（消息接口包，rosidl）：RoboMaster 消息与服务定义，31 个 `.msg` + 4 个 `.srv`，依赖 `std_msgs`/`sensor_msgs`/`geometry_msgs`。该目录是独立 git 仓库（main 分支）。其 README 注明"后续会与 rm_msgs 合并"。注意源码目录名是 `rm-msgs`，但包名是 `rm_msgs`（见 package.xml/CMakeLists）。

### 关键文件

- `src/armor_tracker/src/subscriber.cpp` — 节点主体（`VisionSubscriber`，节点名 `vision_subscriber`，可执行文件名 `subscriber`）：PnP、tf 变换、追踪器生命周期管理（连续 10 帧丢失即删除）、摩尔投票、Marker 发布。含大量中文注释。
- `src/armor_tracker/include/armor_tracker/kalman_filter.hpp` — 4 维状态（x, y, vx, vy）恒速卡尔曼滤波器，header-only，用 OpenCV `cv::Mat` 实现，支持运行时 `setQ`/`setR` 动态调参。
- `src/armor_tracker/launch/tracker_launch.py` — 启动文件：加载 `config/default.yaml` 参数，启动节点 + rviz2（`rviz:=false` 可关闭）。
- `src/armor_tracker/launch/tune_launch.py` — 可视化调参启动文件（配合 `ros2 bag play --clock`）：启动订阅节点 + `image_transport republish`（解压 `/camera/image_raw/compressed` → `/camera/image_raw`）+ rviz2（加载 `config/tune.rviz` 预设布局，含视频面板）。`use_sim_time` 默认为 `true`，接真机时传 `use_sim_time:=false`。
- `src/armor_tracker/config/tune.rviz` — rviz 预设布局：Fixed Frame `gimbal_link`，3D 视图显示 `/armor_markers`，Image 面板显示 `/camera/image_raw`。
- `src/armor_tracker/config/default.yaml` — 卡尔曼参数默认值（`q_pos` / `q_vel` / `r` / `pred_dt`），与 `declare_parameter` 的默认值一致。
- `src/rm-msgs/msg/Detection.msg`、`DetectionArray.msg` — 追踪器的输入消息（每帧检测列表 + 相机内参）。
- 每个包有自己的 `CMakeLists.txt` 和 `package.xml`（colcon/ament 标准配置，无 pyproject.toml/package.json/Cargo.toml）。

## 技术栈

- ROS 2 **Kilted**（`/opt/ros/kilted`），colcon + ament_cmake，CMake ≥ 3.20
- C++（rclcpp、tf2_ros、tf2_geometry_msgs、geometry_msgs、visualization_msgs）
- OpenCV（PnP 解算、卡尔曼滤波矩阵运算）
- Python 仅用于 launch 文件
- 可视化：rviz2

## 构建与运行

```bash
source /opt/ros/kilted/setup.bash
colcon build                      # 构建全部包（rm_msgs 需先于 armor_tracker，colcon 自动处理顺序）
source install/local_setup.bash   # 或 setup.bash

# 运行
ros2 launch armor_tracker tracker_launch.py            # 带 rviz2
ros2 launch armor_tracker tracker_launch.py rviz:=false

# 运行时动态调卡尔曼参数（节点注册了参数回调，修改会应用到所有追踪器）
ros2 param set /vision_subscriber r 0.1
```

运行依赖外部条件：`/vision` 话题（发布 `rm_msgs/DetectionArray`）和 `camera_optical_frame` → `gimbal_link` 的 tf 变换必须由上游节点提供，否则相关检测会被跳过并打印警告。

## 测试

没有单元测试或集成测试代码。两个包都配置了 `ament_lint_auto`（`BUILD_TESTING` 时启用），且都显式跳过了 copyright 和 cpplint 检查：

```bash
colcon test                       # 跑 ament 系列 lint
colcon test-result --verbose
```

## 代码风格与约定

- 注释和文档**主要使用中文**，且注释偏教学式（解释语法和设计意图，如"成员初始化列表——用tracker构造时收到的三个参数去构造成员kf"）。新增/修改代码请延续中文注释风格。
- C++ 编译选项 `-Wall -Wextra -Wpedantic`；K&R 大括号风格在 `kalman_filter.hpp`，紧凑单行风格在 `subscriber.cpp`，修改时跟随所在文件。
- `armor_tracker` 的单头文件库使用 include guard（如 `ARMOR_TRACKER__KALMAN_FILTER_HPP_`）。
- 坐标系约定：检测在 `camera_optical_frame`，追踪/可视化在 `gimbal_link`，Marker 的 frame_id 写死为 `gimbal_link`。
- 身份码编码：`class_id = color * 10 + type`，color ∈ {0:blue, 1:red, 2:white}（见 `Detection.msg` 注释）。
- 关键阈值硬编码在 `subscriber.cpp`：匹配阈值 `match_thresh = 0.45` m，删除阈值 `max_lost = 10` 帧；卡尔曼 Q/R 和预测提前量则通过 ROS 参数可调。

## 注意事项

- 修改 `rm-msgs` 的 `.msg`/`.srv` 后，必须同步把新文件加入其 `CMakeLists.txt` 的 `rosidl_generate_interfaces` 列表并重新 `colcon build`。
- 不要把 `build/`、`install/`、`log/` 纳入版本控制（colcon 产物）。
- `armor_tracker/package.xml` 的 description 仍是 TODO，版本号 0.0.0 —— 项目处于早期开发阶段。
- 无特殊安全敏感内容；无密钥、无 CI 配置、无部署流程（在机器人/本机直接 colcon build 运行）。
