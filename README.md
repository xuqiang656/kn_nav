# kn_nav_ws

Integrated navigation workspace for local/device synchronization.

Source packages are kept under `src/`:

- `FAST_LIO_LOCALIZATION_HUMANOID`
- `FastDEM`
- `PCT_planner`
- `traversability_estimation`
- `art_planner`

The source trees were copied from the existing workspaces without their nested
`.git`, `build`, `install`, `log`, or Python cache directories.

Typical usage:

```bash
cd ~/xq/code/work_space/kn_nav_ws
colcon build --symlink-install
```
