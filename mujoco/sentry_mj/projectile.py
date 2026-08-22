"""17mm 弹丸发射与回收.

物理受力由 MuJoCo 自己算, 不手写弹道:
  - 重力
  - 气动阻力与升力: MJCF 里 option density=1.204 + geom fluidshape="ellipsoid",
    MuJoCo 按椭球模型算阻力、升力、马格努斯力和角阻尼, 比自己写 0.5*rho*Cd*A*v^2
    多了自旋耦合项, 正好用来看弹丸自旋对落点的影响
  - 碰撞: condim=4, 与地面/机器人正常接触

弹丸用对象池: MJCF 里预建 N 个 free body, 发射即改 qpos/qvel, 回收即挪到地下并
清零速度。这样 mjModel 结构固定, 可以直接用于 MJX 批量仿真。
"""
from __future__ import annotations

import math

import mujoco
import numpy as np


class ProjectilePool:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, prefix: str = 'proj'):
        self.m, self.d = model, data
        self.bodies, self.qadr, self.vadr = [], [], []
        i = 0
        while True:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'{prefix}{i}')
            if bid < 0:
                break
            jid = model.body_jntadr[bid]
            self.bodies.append(bid)
            self.qadr.append(model.jnt_qposadr[jid])
            self.vadr.append(model.jnt_dofadr[jid])
            i += 1
        self.n = len(self.bodies)
        self._next = 0
        self._fired_at = np.full(self.n, -1e9)
        self._alive = np.zeros(self.n, dtype=bool)
        self._last_shot = -1e9
        self.muzzle_sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'muzzle')
        if self.n == 0:
            raise RuntimeError('MJCF 里没有弹丸池, 检查 gen_mjcf.py 的 N_PROJECTILE')

    # ---- 内部 ----
    def _park(self, k: int):
        """把弹丸挪到地下并冻结, 不参与可见仿真"""
        qa, va = self.qadr[k], self.vadr[k]
        self.d.qpos[qa:qa + 3] = (0.0, 0.0, -5.0 - 0.1 * k)
        self.d.qpos[qa + 3:qa + 7] = (1.0, 0.0, 0.0, 0.0)
        self.d.qvel[va:va + 6] = 0.0
        self._alive[k] = False

    def reset(self):
        for k in range(self.n):
            self._park(k)
        self._fired_at[:] = -1e9
        self._last_shot = -1e9

    # ---- 对外 ----
    def fire(self, cfg, rng: np.random.Generator | None = None) -> bool:
        """从 muzzle site 发射一发. 冷却未到返回 False。"""
        t = self.d.time
        if t - self._last_shot < cfg.cooldown_s:
            return False
        rng = rng or np.random.default_rng()

        pos = self.d.site_xpos[self.muzzle_sid].copy()
        R = self.d.site_xmat[self.muzzle_sid].reshape(3, 3)
        forward = R[:, 2]                       # site 的 +Z 为出膛方向

        if cfg.spread_deg > 0:
            sig = math.radians(cfg.spread_deg)
            perp1 = R[:, 0]
            perp2 = R[:, 1]
            forward = forward + perp1 * rng.normal(0, sig) + perp2 * rng.normal(0, sig)
            forward /= np.linalg.norm(forward)

        k = self._next
        self._next = (self._next + 1) % self.n
        qa, va = self.qadr[k], self.vadr[k]
        # 出膛点前移一个半径, 避免与枪管几何初始穿透
        self.d.qpos[qa:qa + 3] = pos + forward * 0.012
        self.d.qpos[qa + 3:qa + 7] = (1.0, 0.0, 0.0, 0.0)
        self.d.qvel[va:va + 3] = forward * cfg.muzzle_speed
        # 自旋绕出膛轴, 用于观察马格努斯效应
        spin = cfg.spin_rpm * 2.0 * math.pi / 60.0
        self.d.qvel[va + 3:va + 6] = forward * spin
        self._fired_at[k] = t
        self._alive[k] = True
        self._last_shot = t
        return True

    def update(self, cfg):
        """回收超时或已落到地面以下的弹丸"""
        t = self.d.time
        for k in range(self.n):
            if not self._alive[k]:
                continue
            if t - self._fired_at[k] > cfg.life_s or self.d.qpos[self.qadr[k] + 2] < -0.5:
                self._park(k)

    @property
    def alive_count(self) -> int:
        return int(self._alive.sum())

    def alive_states(self):
        """返回存活弹丸的 (位置, 速度), 供落点统计使用"""
        out = []
        for k in range(self.n):
            if self._alive[k]:
                qa, va = self.qadr[k], self.vadr[k]
                out.append((self.d.qpos[qa:qa + 3].copy(), self.d.qvel[va:va + 3].copy()))
        return out
