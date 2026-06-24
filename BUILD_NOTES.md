# Build Notes

Date: 2026-06-22

Command:

```bash
colcon build --symlink-install
```

## Errors and fixes

1. `planning_core` could not find GTSAM.

   Error:

   ```text
   Could not find a package configuration file provided by "GTSAM"
   GTSAMConfig.cmake
   ```

   Cause: `PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install` is a local third-party build output and was not copied into the new workspace.

   Fix: restored the local GTSAM install directory from the old `PCT_ws`. This directory is ignored by git.

2. `planning_core` could not find OSQP.

   Error:

   ```text
   Could not find a package configuration file provided by "osqp"
   osqpConfig.cmake / osqp-config.cmake
   ```

   Cause: `PCT_planner/planner/lib/3rdparty/osqp/install` is also a local third-party build output and was not copied into the new workspace.

   Fix: restored the local OSQP install directory from the old `PCT_ws`. This directory is ignored by git.

3. `open3d_loc` could not find Open3D.

   Error:

   ```text
   Could not find a package configuration file provided by "Open3D"
   Open3DConfig.cmake
   ```

   Cause: `open3d_loc` only defaulted to `/home/code/thirdparty/open3d141/lib/cmake/Open3D`, while this machine has Open3D at `/home/kangneng/xq/code/thirdparty/open3d141/lib/cmake/Open3D`.

   Fix: updated `open3d_loc/CMakeLists.txt` to keep `-DOpen3D_DIR` and `$Open3D_DIR` first, then try the Docker path, then try the local source-tree relative path.

4. `fastdem_ros` tried to build a ROS 1 catkin package inside the ROS 2 workspace.

   Error:

   ```text
   Could not find a package configuration file provided by "catkin"
   catkinConfig.cmake
   ```

   Cause: `src/FastDEM/ros1` is a ROS 1 package and should not be discovered by ROS 2 colcon.

   Fix: added `src/FastDEM/ros1/COLCON_IGNORE`.

## Final result

Final build completed successfully:

```text
Summary: 13 packages finished [5min 1s]
```

Note: if this workspace is cloned on another machine, rebuild PCT third-party outputs with:

```bash
src/PCT_planner/planner/build_thirdparty.sh
```


已经了解完成，没有修改代码。

当前导航链路是：

```text
MID360 → FAST-LIO → /Odometry_loc、点云
                    ↓
Open3D Localization → /Odometry_open3d + map→base_link TF
                    ↓
PCT Global Planner ← /goal_pose 或 /clicked_point
                    ↓
                /pct_path
                    ↓
Pure Pursuit → /cmd_vel
                    ↓
Go2 Safety Bridge → Unitree SportClient::Move
```

几个关键认识：

- 定位统一在 `map` 坐标系，输出 `/Odometry_open3d`，与 PCT 路径坐标一致。
- PCT 使用 TF 中机器狗当前位置作为起点，发布平滑路径 `/pct_path`。
- Pure Pursuit 当前只跟踪路径的 XY，路径 Z 主要用于 PCT 楼层选择，并未进入机器狗姿态或上下楼控制。
- 控制频率为 10 Hz，当前目标速度配置为 `0.4 m/s`，角速度最终由 bridge 限制在 `±0.5 rad/s`。
- Go2 bridge 默认禁用，需要手动调用 `/go2_cmd_vel_bridge/enable`；定位、`cmd_vel` 或运动状态超时都会停车并锁回禁用状态。
- 目前没有接入局部避障，`obstacle_th` 等参数尚未真正参与控制。
- 完整系统仍分成定位、PCT、跟踪控制三套启动入口，还不是一键 bringup。
- 定位地图和 PCT 地面地图是不同文件，后续必须持续保证两者原点、朝向和尺度完全一致。
- 工作区构建记录显示 13 个包已成功编译；仓库中存在现有未提交的 PCT/Pure Pursuit/Go2 安全桥改动，我已原样保留。

整体上，这已经是“全局定位 → 全局路径 → 二维路径跟踪 → 安全速度下发”的可运行骨架。下一步可以在这个基础上继续排查实机跟踪、坐标一致性、局部避障或整套启动编排。
