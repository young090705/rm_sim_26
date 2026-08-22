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
    def feed(self, cfg, rng: np.random.Generator | None = None) -> int | None:
        """送弹: 把一发放到摩擦轮入口, 给一点推弹速度。

        真车是拨弹盘把弹丸送进两个摩擦轮之间, 由轮子摩擦加速出膛 —— 初速不是
        设定值, 而是轮速的结果。所以这里只负责"喂进去", 加速交给接触求解器。

        返回弹丸槽位; 冷却未到返回 None。
        """
        t = self.d.time
        if t - self._last_shot < cfg.cooldown_s:
            return None
        rng = rng or np.random.default_rng()

        pos = self.d.site_xpos[self.muzzle_sid].copy()
        R = self.d.site_xmat[self.muzzle_sid].reshape(3, 3)
        forward = R[:, 2]

        if cfg.spread_deg > 0:
            sig = math.radians(cfg.spread_deg)
            pos = pos + R[:, 0] * rng.normal(0, sig * 0.004) \
                      + R[:, 1] * rng.normal(0, sig * 0.004)

        k = self._next
        self._next = (self._next + 1) % self.n
        qa, va = self.qadr[k], self.vadr[k]
        # 放在摩擦轮入口后方, 沿膛线推进
        self.d.qpos[qa:qa + 3] = pos - forward * cfg.feed_offset
        self.d.qpos[qa + 3:qa + 7] = (1.0, 0.0, 0.0, 0.0)
        self.d.qvel[va:va + 3] = forward * cfg.feed_speed
        self.d.qvel[va + 3:va + 6] = 0.0
        self._fired_at[k] = t
        self._alive[k] = True
        self._last_shot = t
        return k

    def fire(self, cfg, rng: np.random.Generator | None = None) -> bool:
        """兼容旧接口: 送弹即视为发射"""
        return self.feed(cfg, rng) is not None

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
