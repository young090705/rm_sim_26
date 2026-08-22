"""云台与底盘控制. 这里的增益是辨识前的初值, 不是最终参数。"""
from __future__ import annotations

import math

import mujoco
import numpy as np

GIMBAL_JOINTS = ('big_yaw_joint', 'small_yaw_joint', 'pitch_joint')
WHEEL_JOINTS = ('wheel_fl_joint', 'wheel_rl_joint', 'wheel_rr_joint', 'wheel_fr_joint')
# 轮位角, 与 scripts/omni_drive.py 保持一致
WHEEL_ANGLE = (math.radians(45), math.radians(135), math.radians(-135), math.radians(-45))
WHEEL_SIGN = -1.0


class Controller:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.m, self.d = model, data
        self.jid = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
                    for n in GIMBAL_JOINTS + WHEEL_JOINTS}
        self.aid = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{n}_trq')
                    for n in GIMBAL_JOINTS + WHEEL_JOINTS}
        self.target = {n: 0.0 for n in GIMBAL_JOINTS}
        self.cmd_vel = np.zeros(3)          # vx, vy, wz (机体系)
        self._scan_t = 0.0
        self.manual_torque = {}             # 辨识时直接给力矩, 绕过位置环

    # ---- 状态 ----
    def qpos(self, joint: str) -> float:
        return float(self.d.qpos[self.m.jnt_qposadr[self.jid[joint]]])

    def qvel(self, joint: str) -> float:
        return float(self.d.qvel[self.m.jnt_dofadr[self.jid[joint]]])

    # ---- 云台 ----
    def _gimbal_gains(self, cfg):
        g = cfg.gimbal
        return {'big_yaw_joint': (g.big_yaw_kp, g.big_yaw_kd),
                'small_yaw_joint': (g.small_yaw_kp, g.small_yaw_kd),
                'pitch_joint': (g.pitch_kp, g.pitch_kd)}

    def update_scan(self, cfg, dt: float):
        """无目标时的巡航. 与 at_vision_simulator 的 sentry_scan 同思路。"""
        if not cfg.gimbal.scan_enabled:
            return
        self._scan_t += dt
        self.target['big_yaw_joint'] += cfg.gimbal.scan_yaw_speed * dt
        lo, hi = cfg.gimbal.scan_pitch_range
        span = hi - lo
        if span > 0:
            phase = (self._scan_t * cfg.gimbal.scan_pitch_speed) % (2 * span)
            self.target['pitch_joint'] = lo + (phase if phase < span else 2 * span - phase)

    def apply_gimbal(self, cfg):
        gains = self._gimbal_gains(cfg)
        for j in GIMBAL_JOINTS:
            a = self.aid[j]
            if a < 0:
                continue
            if j in self.manual_torque:
                self.d.ctrl[a] = self.manual_torque[j]
                continue
            kp, kd = gains[j]
            err = self.target[j] - self.qpos(j)
            if j == 'big_yaw_joint':                 # 无限位, 取最短路径
                err = (err + math.pi) % (2 * math.pi) - math.pi
            self.d.ctrl[a] = kp * err - kd * self.qvel(j)

    # ---- 底盘 ----
    def apply_chassis(self, cfg):
        c = cfg.chassis
        vx, vy, wz = self.cmd_vel
        for j, theta in zip(WHEEL_JOINTS, WHEEL_ANGLE):
            a = self.aid[j]
            if a < 0:
                continue
            tangential = -vx * math.sin(theta) + vy * math.cos(theta) + wz * c.mount_radius
            w_target = WHEEL_SIGN * tangential / c.wheel_radius
            self.d.ctrl[a] = np.clip(c.drive_kp * (w_target - self.qvel(j)),
                                     -c.max_wheel_torque, c.max_wheel_torque)

    def step(self, cfg, dt: float):
        self.update_scan(cfg, dt)
        self.apply_gimbal(cfg)
        self.apply_chassis(cfg)
