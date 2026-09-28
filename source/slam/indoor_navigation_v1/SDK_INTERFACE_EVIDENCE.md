# S10 高层 SDK 核对记录

核对日期：2026-09-13。来源：本地《S10软件开发指南202607.pdf》第 50–53 页，以及机器人 ROS 2 domain 0 的只读发现/类型描述服务。

## 选用接口

- `/NAV_CMD`，`drdds/msg/NavCmd`，10 Hz，x/y 为 m/s，yaw 为 rad/s；x 前、y 左、yaw 逆时针为正。
- 只在导航模式生效，依赖 `basic_server`。原机 `planner`、`charge_manager` 会竞争速度发布权。
- 不使用 `/STEER`：它在已部署的 RL 接口中是归一化比例，不是这里要求的米/秒。
- 不发布 `/JOINTS_CMD`，不启动另一份 RL/关节策略，不自动切换模式或步态。

## 在线消息结构纠正

从 `/traversability_estimation_node/get_type_description` 读取了 `drdds/msg/MotionInfo` 的真实定义。Jazzy 提供只读类型描述服务；背景见 [ROS 官方类型描述说明](https://docs.ros.org/en/iron/Releases/Release-Jazzy-Jalisco.html)。SDK 网关使用两个明确的 ROS context 和各自的 executor，见 [rclpy Context 文档](https://docs.ros.org/en/jazzy/p/rclpy/api/context.html)。

在线及本地编译得到的 MotionInfo 哈希一致：

`RIHS01_a7d7f26e0f1152b10643f0e8bed1c9fd7b7f574170f3460335e381c40070830b`

在线真实 MetaType 使用 `builtin_interfaces/Time stamp`。MotionInfoValue 中字段依次为 `vel_x, vel_y, vel_yaw, height, motion_state, gait_state, payload, remain_mile`；PDF 省略了部分嵌套字段名和 `remain_mile`，不能直接照排版生成。

NavCmd 根据 PDF 的三速度字段及在线一致的 MetaType 生成。原生端点是 bare DDS，未提供 NavCmd 类型哈希；本轮没有收到原机实际 NavCmd 数据，尚未验证真机速度响应。隔离域测试验证的是自建消息包的 ROS 传输与零速保护，不能代替此项真机验收。

## 已观察到的阻止条件

- `/NAV_CMD` 有 2 个现有发布者和 1 个接收者。
- `rl_deploy.service` 活跃，用户 xxx，`/JOINTS_CMD` 有 `rl_deploy` 发布者。
- `/dev/input/js0` 不存在。
- 只读 MotionInfo：state=0、gait=0、速度 0，height 约 0.071 m；不是可执行本试验的 RL 平地状态。
- CHARGE_STATUS 为 0/error 0，约 1 Hz。因此充电反馈超时为 1.5 s，而不是错误地设成小于发布周期。

本轮没有停止/禁用这些原机服务，没有修改 xxx/ysc，没有向原生 domain 0 发布任何速度或关节指令。

## 软件保护的边界

失去定位、前向感知、双雷达健康、SDK 反馈、操作员续期或独占控制权时，网关锁定零速；恢复数据不自动重启行走。程序正常退出发送零速脉冲。

网关被 SIGKILL、Orin 断电、链路中断时无法继续发零速。必须由现场验证 S10 接收端的超时停车和物理急停，不能拿应用层计时器冒充接收端安全机制。软件不提供绕过这些确认的自动启动。
