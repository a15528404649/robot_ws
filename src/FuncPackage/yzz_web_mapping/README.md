# YZZ Windows 网页建图与导航

`yzz_web_mapping` 是运行在 Orin 上的本地 Web 后端和浏览器前端。Windows 只需浏览器，不需要安装 ROS 2。

## 架构

```text
Windows browser -> login page -> Orin HTTP/ROS2 backend -> existing SLAM / AMCL / Nav2
```

该包不会修改或自动启动既有的建图、导航系统；只有网页中点击启动后，才会调用已有的 `mapping_system.launch.py` 或 `navigation_system.launch.py`。

## 首次登录配置

账号配置不保存在源码中。密码会使用 PBKDF2 哈希写到：

```text
~/.config/yzz_web_mapping/auth.json
```

该目录权限为 `700`，文件权限为 `600`。以后重设账号或密码：

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run yzz_web_mapping configure_auth --username <账号>
```

工具会交互式询问新密码。

## 启动网页服务

```bash
cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch yzz_web_mapping web_mapping.launch.py
```

Windows 与 Orin 位于同一局域网后，在 Windows 浏览器打开：

```text
http://<Orin-IP>:8080
```

当前 Orin Wi-Fi 地址可用 `hostname -I` 查看。服务启动期间不要关闭该 Orin 终端。

## Windows 操作流程

1. 打开网址，使用配置好的账号登录。
2. 点击“启动建图系统”。网页会启动现有底盘、IMU、雷达、EKF 与 `slam_toolbox` 建图流程。
3. 驾驶机器人完成建图；页面会显示 `/map`。
4. 在“保存地图”填写名称，例如 `office_01`，点击保存。地图保存到 `yzz_navigation2/maps/`，生成 `.yaml` 和 `.pgm`。
5. 停止本网页启动的建图系统。
6. 在“导航”输入已有地图名称，例如 `office_01`，点击“启动导航”。
7. 选择地图工具栏中的“初始位姿”，在地图上按住并拖拽，发布 AMCL 初始位姿与朝向；当状态显示 `amcl` 后，选择“2D Goal”或“Navigation Goal”并拖拽下发目标。
8. 建图时可按住网页方向键，或使用键盘 W/A/S/D 遥控机器人。

## 多点巡航操作

多点巡航任务始终在 Orin 本机执行；Windows 浏览器只负责编辑路线和发送控制请求，不需要虚拟机直接发布 ROS 话题或 Action。

1. 在“导航”中选择已保存地图，并点击“用所选地图启动导航”。
2. 选择“初始位姿”，在地图上按住并拖拽，完成 AMCL 当前定位；看到定位来源为 `amcl` 后再继续。
3. 点击地图工具栏的“添加航点”，在地图中按住并拖拽：起点为航点位置，拖拽方向为机器人到点后的朝向。可连续添加多个航点。
4. 在“多点巡航”输入路线名称，例如 `daily_patrol`，点击“保存当前航点”。路线会保存到：

```text
~/robot_ws/data/waypoints/<地图名>/<路线名>.json
```

5. 点击“开始巡逻”。Orin 的 `yzz_waypoint_nav` 节点会依次调用 Nav2；页面可显示当前巡逻状态和航点进度。
6. 使用“暂停 / 继续 / 停止”管理任务。下次只需选择相同地图和已保存路线，无需重新添加航点，但仍必须设置机器人当前初始位姿。

路线文件与地图分开保存：地图保存环境，路线保存任务目标；因此同一张地图可有多条巡逻路线。

## 安全限制

- 未登录时，地图、建图、导航、保存地图与控制 API 均不可访问。
- 网页仅适合受信任的局域网，不应直接暴露到公网。
- 网页遥控默认启用，限速为线速度 0.20 m/s、角速度 0.70 rad/s；方向按钮或 W/A/S/D 必须持续按住才会续发命令，松开、切换页面或 0.4 秒未续发都会自动停车。Nav2 导航运行时网页遥控会自动锁定，避免抢占 `/cmd_vel`。
- 网页只能停止由网页自身启动的建图或导航进程，不会杀掉你在其他终端手动启动的系统。

## 后续一键配置

可在未来的安装脚本中依次执行依赖安装、工作空间构建、`configure_auth` 初始化和网页服务注册。账号密码应从交互输入或部署环境变量提供，不应写进 Git 或 ROS 源码。
