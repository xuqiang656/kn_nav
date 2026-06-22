import os
import sys
import pickle
import numpy as np
from scipy.ndimage import distance_transform_edt

from utils import *

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomogramPlanner(object):
    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate
        self.a_star_cost_threshold = self.cfg.planner.a_star_cost_threshold
        self.optimizer_cost_threshold = getattr(
            self.cfg.planner,
            'optimizer_cost_threshold',
            getattr(self.cfg.planner, 'safe_cost_margin', 10.0)
        )
        self.step_cost_weight = self.cfg.planner.step_cost_weight
        self.use_clearance_cost = getattr(self.cfg.planner, 'use_clearance_cost', True)
        self.clearance_cost_weight = getattr(self.cfg.planner, 'clearance_cost_weight', 8.0)
        self.clearance_cost_decay = getattr(self.cfg.planner, 'clearance_cost_decay', 1.0)

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)
        self.elev_g = None
        self.raw_trav = None
        self.planning_trav = None
        self.clearance = None
        self.last_astar_path_3d = None

    def loadTomogram(self, tomo_file):
        with open(self.tomo_dir + tomo_file + '.pickle', 'rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        elev_g = tomogram[3]
        elev_g = np.nan_to_num(elev_g, nan=-100)
        self.elev_g = elev_g
        elev_c = tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)

        planning_trav, planning_grad_x, planning_grad_y = self.buildPlanningCost(
            trav, elev_g
        )
        self.initPlanner(
            trav, planning_trav, planning_grad_x, planning_grad_y, elev_g,
            elev_c
        )
        
    def initPlanner(self, trav, planning_trav, planning_grad_x, planning_grad_y,
                    elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        self.planner.init_map(
            self.a_star_cost_threshold, self.optimizer_cost_threshold,
            self.resolution, self.n_slice, self.step_cost_weight,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            planning_trav.reshape(-1, planning_trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            planning_grad_y.reshape(-1, planning_grad_y.shape[-1]).astype(np.double),
            -planning_grad_x.reshape(-1, planning_grad_x.shape[-1]).astype(np.double)
        )

    def plan(self, start_pos, end_pos):
        # TODO: calculate slice index. By default the start and end pos are all at slice 0
        self.start_idx[1:] = self.pos2idx(start_pos)
        self.end_idx[1:] = self.pos2idx(end_pos)

        self.planner.plan(self.start_idx, self.end_idx, True)
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            self.last_astar_path_3d = None
            return None
        self.last_astar_path_3d = self.pathGridToMap(path)

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d

    def getLastAstarPath(self):
        return self.last_astar_path_3d

    def buildPlanningCost(self, trav, elev_g):
        self.raw_trav = trav.copy()
        planning_trav = trav.astype(np.float32).copy()
        self.clearance = np.zeros_like(planning_trav, dtype=np.float32)

        if self.use_clearance_cost:
            valid_floor = elev_g > -50
            passable = valid_floor & (trav <= self.a_star_cost_threshold)
            for layer in range(trav.shape[0]):
                clearance = (
                    distance_transform_edt(passable[layer]).astype(np.float32)
                    * self.resolution
                )
                clearance_cost = self.clearance_cost_weight * np.exp(
                    -clearance / max(self.clearance_cost_decay, 1e-3)
                )
                clearance_cost[~passable[layer]] = 0.0
                planning_trav[layer] += clearance_cost.astype(np.float32)
                self.clearance[layer] = clearance

        grad_x = np.zeros_like(planning_trav, dtype=np.float32)
        grad_y = np.zeros_like(planning_trav, dtype=np.float32)
        grad_x[:, 1:-1, :] = (planning_trav[:, 2:, :] - planning_trav[:, :-2, :]) * 0.5
        grad_y[:, :, 1:-1] = (planning_trav[:, :, 2:] - planning_trav[:, :, :-2]) * 0.5
        self.planning_trav = planning_trav
        return planning_trav, grad_x, grad_y

    def pathGridToMap(self, path):
        layers = path[:, 0].astype(np.int32)
        rows = path[:, 1].astype(np.int32)
        cols = path[:, 2].astype(np.int32)
        heights = self.elev_g[layers, rows, cols]
        astar_grid = np.stack([cols, rows, heights / self.resolution], axis=1)
        return transTrajGrid2Map(self.map_dim, self.center, self.resolution, astar_grid)
    
    def pos2idx(self, pos):
        pos = pos - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        idx = np.array([idx[1], idx[0]], dtype=np.float32)
        return idx
