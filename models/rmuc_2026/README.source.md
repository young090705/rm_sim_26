# RMUC 2026 场地查看

来源：https://github.com/git-alphabet/simulation
提交：11561067834255e7b15bf0be89de2deaa15e9c07
原始路径：src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource

社区场地模型，灰色 STL 材质；未核验与官方最终场地的一致性。
保留原始 STL、缩放和场地安装位姿，Harmonic 版本替换了 Gazebo 系统插件名称。

`worlds/RMUC2026_world.world` 另外增加了静态防跌落围墙 `field_boundary_walls`。
按 STL 底板的实际边界布置：世界坐标 X 为 `[-14.505, 14.545] m`，
Y 为 `[-7.400656, 8.649344] m`，底板上表面 Z 为 `0.358656 m`。
墙厚 `0.2 m`，高出底板 `2 m`，四角搭接闭合；每面墙均包含碰撞体和可见外观，
会参与雷达扫描。围墙是仿真防护设施，不属于原始场地 STL。

启动：bash ~/rmoss_ws/maps/rmuc2026/view_map.sh
