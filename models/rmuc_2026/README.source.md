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

## 地面碰撞

原始 STL 包含普通底板和毫米级实体标线。直接把整幅 STL 用作碰撞体时，
出生点附近可复现斜向移动突然偏航、轮子转动但车体停住的问题。

外观和雷达仍使用原始 `rmuc_2026.stl`；物理碰撞分为：

- `flat_floor`：等尺寸长方体底板，普通地面上表面世界高度仍为 `0.358656 m`。
- `field_structures`：`rmuc_2026_collision.stl`，保留斜坡、台阶、起伏路段及场地结构。
  仅移除完整底板和普通地面/高出底板 `0.2 m` 平台上的薄片标线；
  薄片按独立连通件识别，厚度不超过 `5 mm`、顶部距对应地面不超过 `6 mm`。

重新生成碰撞网格（依赖 NumPy）：

```bash
python3 scripts/cad/simplify_rmuc2026_collision.py
colcon build --packages-select rm_sim_26
```

生成时不会移动保留的网格顶点。毫米级标线保留视觉/雷达外观，但不再与轮子碰撞。

启动：bash ~/rmoss_ws/maps/rmuc2026/view_map.sh
