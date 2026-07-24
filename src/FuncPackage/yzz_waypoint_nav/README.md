# YZZ ROS2 多点巡航导航

`yzz_waypoint_nav` 是运行在 Orin Jazzy 上的多点巡航执行节点，ROS 节点名为 `waypoint_nav_node`。它保留 ROS1 `yzz_waypoint_nav` 的顺序巡逻、循环、随机、到点停留和超时语义，但使用 ROS2 Nav2 的 `NavigateToPose` Action。

## 安全边界

- 不直接发布 `/cmd_vel`；路径规划、避障和底盘速度控制仍完全由 Nav2 负责。
- 节点启动后保持 `idle`，不会自动开始巡逻。
- 只有网页或其他受控客户端向 `/yzz_waypoint_nav/command` 发送 `start`，且 Nav2 Action 已就绪时，才会发送第一个导航目标。

## 路线数据

路线按地图隔离保存，位置为：

```text
~/robot_ws/src/FuncPackage/yzz_waypoint_nav/data/waypoints/<地图名>/<路线名>.json
```

每个航点包含 `x`、`y`、`yaw` 和显示名称。地图名和路线名仅允许字母、数字、`_`、`-`。

## ROS1 参数兼容

保留参数：`rest_time`、`keep_patrol`、`random_patrol`、`patrol_type`、`patrol_loop`、`patrol_time`、`potrol_points_num`、`goal_timeout`。

其中 `potrol_points_num` 保留原来的拼写；默认 `0` 表示执行已保存路线的全部航点。

## 单独启动

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch yzz_waypoint_nav yzz_waypoint_nav.launch.py
```

通常无需单独运行：`ros2 launch yzz_web_mapping web_mapping.launch.py` 会同时启动网页和空闲的多点巡航节点。
