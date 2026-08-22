"""交互式查看器: 多视角切换、发射弹丸、云台与底盘操控.

视角设计参考 at_vision_simulator 的 Free / FirstPerson / ThirdPerson 循环, 这里扩成
五个: 自由 + 三个跟随机位 + 两个机载相机(云台自瞄相机、雷达位)。

按键 (避开 MuJoCo 内置的 Space/Tab/Backspace/[]/Esc/F1-F5/方向键)
  V            循环切换视角
  1..6         直接选视角
  X            发射一发
  B            连发开关
  N            摩擦轮启停
  R            复位
  P            打印状态(位姿、关节角、轮速、弹丸数)
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

# 弹丸进入摩擦轮多近就开始降步长。摩擦轮半径 30mm, 取 70mm 覆盖接近段与咬合段。
FINE_STEP_RADIUS = 0.070

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
        self._base_dt = float(self.m.opt.timestep)
        self._fric_bids = [b for b in
                           (mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, n)
                            for n in ('fric_up', 'fric_dn')) if b >= 0]
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

    # ---- 发射 ----
    def fire_one(self) -> bool:
        """送一发弹进摩擦轮. 加速交给接触求解器, 初速是结果不是输入.

        摩擦轮没转到位就不打 —— 真车同理, 否则初速不足。
        """
        if not self.ctrl.fric_ready(self.cfg):
            return False
        return self.pool.feed(self.cfg.projectile, self.rng) is not None

    # ---- 输入 ----
    # Space / Tab / Backspace / [ ] / Esc / F1-F5 / 方向键 由 MuJoCo 查看器占用,
    # 按下去不会传到这里, 所以自定义功能一律避开它们。
    def on_key(self, keycode: int):
        ch = chr(keycode).upper() if 32 <= keycode < 127 else ''
        if ch == 'V':                       # 切视角
            self.view_idx = (self.view_idx + 1) % len(VIEWS)
            print(f'[视角] {self.view_idx + 1}/{len(VIEWS)} {VIEWS[self.view_idx][0]}')
        elif ch in '123456':
            i = int(ch) - 1
            if i < len(VIEWS):
                self.view_idx = i
                print(f'[视角] {VIEWS[i][0]}')
        elif ch == 'X':                     # 单发
            if self.fire_one():
                print(f'[发射] 存活 {self.pool.alive_count}')
            else:
                w = abs(self.ctrl.qvel('fric_up_joint'))
                print(f'[未发射] 摩擦轮 {w:.0f}/{self.cfg.projectile.fric_speed:.0f} rad/s')
        elif ch == 'B':                     # 连发
            self.firing = not self.firing
            print(f'[连发] {"开" if self.firing else "关"}')
        elif ch == 'N':                     # 摩擦轮开关
            self.ctrl.fric_on = not self.ctrl.fric_on
            print(f'[摩擦轮] {"启动" if self.ctrl.fric_on else "停转"}')
        elif ch == 'R':
            self.reset()
            print('[复位]')
        elif ch == 'P':
            self.dump()

    def dump(self):
        print(f'--- t={self.d.time:6.2f}s ---')
        print(f'  base xyz = {np.round(self.d.qpos[0:3], 4)}')
        for j in GIMBAL_JOINTS:
            print(f'  {j:16s} q={math.degrees(self.ctrl.qpos(j)):8.2f}deg '
                  f'v={self.ctrl.qvel(j):7.3f}rad/s')
        for j in ('fric_up_joint', 'fric_dn_joint'):
            print(f'  {j:16s} w={self.ctrl.qvel(j):8.1f} rad/s')
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

                if self.firing:
                    self.fire_one()
                self._advance(dt * steps_per_frame)
                self.pool.update(self.cfg.projectile)

                self._apply_view(v)
                v.sync()
                if realtime:
                    lag = self.d.time - (time.perf_counter() - wall0)
                    if lag > 0:
                        time.sleep(lag)

    def _near_friction_wheels(self) -> bool:
        """是否有弹丸正在摩擦轮附近 —— 用它决定要不要降步长。

        起初用的是"发射后固定 12 ms 窗口", 但弹丸以送弹速度爬行约 10 ms 才被
        两轮咬住, 咬合时刻恰好落在窗口边缘: 有的发在窗内(得到收敛值 18.5 m/s),
        有的漏到窗外(退回粗步长的错误值 24 m/s), 同一份配置打出两种初速。
        改成按距离判定就与时序无关了。
        """
        for wid in self._fric_bids:
            wpos = self.d.xpos[wid]
            for k in range(self.pool.n):
                if not self.pool._alive[k]:
                    continue
                p = self.d.qpos[self.pool.qadr[k]:self.pool.qadr[k] + 3]
                if np.linalg.norm(p - wpos) < FINE_STEP_RADIUS:
                    return True
        return False

    def _advance(self, horizon: float):
        """推进 horizon 秒的物理. 弹丸经过摩擦轮时自动降步长。

        轮速 855 rad/s 时 dt=1ms 每步转 52°、接触点划过 27mm(比弹丸直径还大),
        摩擦求解把弹丸当成焊在轮面上, 出膛速度偏高 36%。步长收敛检验显示每步
        转角降到 2° 以下才进入收敛区, 故只在弹丸过轮时改用 launch_dt。
        """
        base_dt = self._base_dt
        end = self.d.time + horizon
        while self.d.time < end - 1e-12:
            step = (self.cfg.projectile.launch_dt if self._near_friction_wheels()
                    else base_dt)
            step = min(step, end - self.d.time)
            if step <= 0:
                break
            self.m.opt.timestep = step
            self.ctrl.step(self.cfg, step)
            mujoco.mj_step(self.m, self.d)
        self.m.opt.timestep = base_dt

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
    print('X 发射   B 连发   V 切视角   N 摩擦轮   R 复位   P 状态')
    print('(Space/Tab 等归 MuJoCo 查看器自身使用)')
    sim.run(duration=args.duration, render_hz=args.render_hz)

    # GLFW 在部分 NVIDIA 驱动上关窗时会段错误(X 连接名被写坏), 与模型无关 ——
    # 最简模型同样复现。仿真本身已跑完, 这里跳过会崩的清理路径直接退出。
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
