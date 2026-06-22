#!/usr/bin/env python3
"""Standalone planner runner (no ROS required)."""
import sys
import os
import argparse
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
root = os.path.abspath(os.path.join(script_dir, '..'))
gtsam_lib = os.path.join(root, 'lib/3rdparty/gtsam-4.1.1/install/lib')
smoothing_lib = os.path.join(root, 'lib/build/src/common/smoothing')


def ensure_runtime_library_path():
    paths = [gtsam_lib, smoothing_lib]
    current = os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep)
    missing = [path for path in paths if path not in current]
    if not missing:
        return

    os.environ['LD_LIBRARY_PATH'] = os.pathsep.join(
        [path for path in current if path] + missing
    )
    os.execvpe(sys.executable, [sys.executable] + sys.argv, os.environ)


ensure_runtime_library_path()

sys.path.insert(0, root)
sys.path.insert(0, os.path.join(root, 'lib'))
from config import Config
from planner_wrapper import TomogramPlanner

parser = argparse.ArgumentParser()
parser.add_argument('--tomo', type=str, default='clinic', help='Tomogram file stem in rsc/tomogram/')
parser.add_argument('--start', type=float, nargs=2, default=[0.0, 0.0], metavar=('X', 'Y'))
parser.add_argument('--end',   type=float, nargs=2, default=[10.0, 5.0], metavar=('X', 'Y'))
args = parser.parse_args()

cfg = Config()
planner = TomogramPlanner(cfg)
planner.loadTomogram(args.tomo)

start_pos = np.array(args.start, dtype=np.float32)
end_pos   = np.array(args.end,   dtype=np.float32)

print(f"[INFO] Planning from {start_pos} to {end_pos}")
traj_3d = planner.plan(start_pos, end_pos)

if traj_3d is None:
    print("[WARN] No path found between the given start and end positions.")
else:
    print(f"[INFO] Trajectory found: {traj_3d.shape[0]} waypoints")
    print("[INFO] First waypoint:", traj_3d[0])
    print("[INFO] Last  waypoint:", traj_3d[-1])
    out = script_dir + '/../../rsc/clinic_traj.npy'
    np.save(out, traj_3d)
    print(f"[INFO] Trajectory saved to {out}")
