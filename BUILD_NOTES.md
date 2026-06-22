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
