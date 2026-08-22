"""运行期配置, 支持热重载.

参考 at_vision_simulator 的做法: 仿真运行时保存 config.toml 即刻生效, 不用重启。
辨识实验里要反复改初速、阻力系数、云台增益, 热重载省掉大量重启时间。
"""
from __future__ import annotations

import os
import tomllib
from dataclasses import dataclass, field, replace

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_PATH = os.path.normpath(os.path.join(HERE, '..', 'config.toml'))


@dataclass(frozen=True)
class Projectile:
    """17mm 弹丸. 质量与半径由 MJCF 固定, 这里是发射参数。"""
    muzzle_speed: float = 25.0        # m/s, 裁判系统 17mm 常见限速
    spin_rpm: float = 0.0             # 出膛自旋, 用于观察马格努斯效应
    spread_deg: float = 0.15          # 散布(高斯 sigma)
    life_s: float = 6.0               # 存活时间, 超时回收
    cooldown_s: float = 0.10          # 射击间隔


@dataclass(frozen=True)
class Gimbal:
    """云台位置环增益. 辨识前的初值, 不是最终参数。"""
    big_yaw_kp: float = 40.0
    big_yaw_kd: float = 4.0
    small_yaw_kp: float = 8.0
    small_yaw_kd: float = 0.5
    pitch_kp: float = 6.0
    pitch_kd: float = 0.4
    scan_enabled: bool = False
    scan_yaw_speed: float = 0.4       # rad/s
    scan_pitch_range: tuple = (-0.2, 0.35)
    scan_pitch_speed: float = 0.5


@dataclass(frozen=True)
class Chassis:
    wheel_radius: float = 0.076
    mount_radius: float = 0.245
    max_wheel_torque: float = 6.0
    drive_kp: float = 1.2             # 轮速环 P


@dataclass(frozen=True)
class Config:
    projectile: Projectile = field(default_factory=Projectile)
    gimbal: Gimbal = field(default_factory=Gimbal)
    chassis: Chassis = field(default_factory=Chassis)
    path: str = DEFAULT_PATH
    _mtime: float = 0.0

    @staticmethod
    def load(path: str = DEFAULT_PATH) -> 'Config':
        if not os.path.exists(path):
            return Config(path=path)
        try:
            with open(path, 'rb') as fh:
                raw = tomllib.load(fh)
        except (OSError, tomllib.TOMLDecodeError) as exc:
            print(f'[config] 解析失败, 沿用当前参数: {exc}')
            return Config(path=path)
        return Config(
            projectile=Projectile(**raw.get('projectile', {})),
            gimbal=Gimbal(**{k: (tuple(v) if isinstance(v, list) else v)
                             for k, v in raw.get('gimbal', {}).items()}),
            chassis=Chassis(**raw.get('chassis', {})),
            path=path,
            _mtime=os.path.getmtime(path),
        )

    def reload_if_changed(self) -> 'Config':
        """文件改动则重新加载, 否则原样返回. 解析失败时保留旧值。"""
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            return self
        if mtime <= self._mtime:
            return self
        new = Config.load(self.path)
        print('[config] 已重载')
        return replace(new, _mtime=mtime)
