"""交互式查看器: 多视角切换、发射弹丸、云台与底盘操控.

视角设计参考 at_vision_simulator 的 Free / FirstPerson / ThirdPerson 循环, 这里扩成
五个: 自由 + 三个跟随机位 + 两个机载相机(云台自瞄相机、雷达位)。

按键
  Tab          循环切换视角
  1..6         直接选视角
  Space        发射一发
  F            按住连发
  W/S/A/D      底盘平移      Q/E 底盘自转
  方向键        云台 yaw / pitch
  R            复位
  P            打印当前状态(位姿、关节角、弹丸数)
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

from .config import Config
from .controller import GIMBAL_JOINTS, Controller
from .projectile import ProjectilePool

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.normpath(os.path.join(HERE, '..', 'models', 'sentry.xml'))

# (显示名, 相机名或 None=自由视角)
VIEWS = [
    ('自由视角', None),
    ('跟随-后方', 'track_chase'),
    ('跟随-侧方', 'track_side'),
    ('跟随-俯视', 'track_top'),
    ('云台相机', 'gimbal_cam'),
    ('雷达视角', 'lidar_cam'),
]


class Sim:
    def __init__(self, model_path: str = MODEL, config_path: str | None = None):
        self.m = mujoco.MjModel.from_xml_path(model_path)
        self.d = mujoco.MjData(self.m)
        self.cfg = Config.load(config_path) if config_path else Config.load()
        self.ctrl = Controller(self.m, self.d)
        self.pool = ProjectilePool(self.m, self.d)
        self.rng = np.random.default_rng(0)
        self.view_idx = 1
        self.firing = False
        self._keys = set()
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.m, self.d)
        # 让底盘落在地面上: base_link 原点距地 0.096
        self.d.qpos[2] = 0.096
        self.pool.reset()
        for j in GIMBAL_JOINTS:
            self.ctrl.target[j] = 0.0
        self.ctrl.cmd_vel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)

    # ---- 输入 ----
    def on_key(self, keycode: int):
        ch = chr(keycode) if 32 <= keycode < 127 else ''
        c = self.cfg
        if keycode == 258:                     # Tab
            self.view_idx = (self.view_idx + 1) % len(VIEWS)
            print(f'[视角] {VIEWS[self.view_idx][0]}')
        elif ch in '123456':
            i = int(ch) - 1
            if i < len(VIEWS):
                self.view_idx = i
                print(f'[视角] {VIEWS[i][0]}')
        elif keycode == 32:                    # Space
            if self.pool.fire(c.projectile, self.rng):
                print(f'[发射] 存活 {self.pool.alive_count}')
        elif ch in 'Ff':
            self.firing = not self.firing
            print(f'[连发] {"开" if self.firing else "关"}')
        elif ch in 'Rr':
            self.reset()
            print('[复位]')
        elif ch in 'Pp':
            self.dump()

    def dump(self):
        print(f'--- t={self.d.time:6.2f}s ---')
        print(f'  base xyz = {np.round(self.d.qpos[0:3], 4)}')
        for j in GIMBAL_JOINTS:
            print(f'  {j:16s} q={math.degrees(self.ctrl.qpos(j)):8.2f}deg '
                  f'v={self.ctrl.qvel(j):7.3f}rad/s')
        print(f'  弹丸存活 {self.pool.alive_count}/{self.pool.n}')

    def _poll_hold(self, viewer):
        """MuJoCo 被动查看器只给按下事件, 持续操作用目标值累积实现"""
        pass

    # ---- 主循环 ----
    def run(self, duration: float | None = None, realtime: bool = True,
            render_hz: float = 60.0):
        """物理与渲染解耦: 物理跑 1 kHz, 画面 60 Hz.

        每个物理步都 sync() 会让帧率被渲染拖到 43 Hz —— 本模型 84 个 geom、
        93 万面, 单帧渲染远比单步物理贵。按 render_hz 折算出每帧的物理步数,
        实时性与画面都正常。
        """
        dt = self.m.opt.timestep
        steps_per_frame = max(1, int(round(1.0 / (render_hz * dt))))
        last_cfg_check = 0.0
        with mujoco.viewer.launch_passive(
                self.m, self.d, key_callback=self.on_key,
                show_left_ui=True, show_right_ui=False) as v:
            self._apply_view(v)
            wall0 = time.perf_counter()
            while v.is_running():
                if duration and self.d.time > duration:
                    break
                if self.d.time - last_cfg_check > 0.4:
                    self.cfg = self.cfg.reload_if_changed()
                    last_cfg_check = self.d.time

                for _ in range(steps_per_frame):
                    if self.firing:
                        self.pool.fire(self.cfg.projectile, self.rng)
                    self.ctrl.step(self.cfg, dt)
                    mujoco.mj_step(self.m, self.d)
                self.pool.update(self.cfg.projectile)

                self._apply_view(v)
                v.sync()
                if realtime:
                    lag = self.d.time - (time.perf_counter() - wall0)
                    if lag > 0:
                        time.sleep(lag)

    def _apply_view(self, v):
        name, cam = VIEWS[self.view_idx]
        if cam is None:
            v.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        else:
            cid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_CAMERA, cam)
            if cid >= 0:
                v.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                v.cam.fixedcamid = cid


def main():
    ap = argparse.ArgumentParser(description='哨兵 MuJoCo 仿真')
    ap.add_argument('--model', default=MODEL)
    ap.add_argument('--config', default=None)
    ap.add_argument('--duration', type=float, default=None)
    ap.add_argument('--view', type=int, default=1, help='初始视角 1..6')
    ap.add_argument('--render-hz', type=float, default=60.0,
                    help='画面刷新率; 物理恒为 1/timestep')
    args = ap.parse_args()

    sim = Sim(args.model, args.config)
    sim.view_idx = max(0, min(args.view - 1, len(VIEWS) - 1))
    print('视角:', '  '.join(f'{i + 1}={n}' for i, (n, _) in enumerate(VIEWS)))
    print('Space 发射   F 连发   Tab 切视角   R 复位   P 打印状态')
    sim.run(duration=args.duration, render_hz=args.render_hz)

    # GLFW 在部分 NVIDIA 驱动上关窗时会段错误(X 连接名被写坏), 与模型无关 ——
    # 最简模型同样复现。仿真本身已跑完, 这里跳过会崩的清理路径直接退出。
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
