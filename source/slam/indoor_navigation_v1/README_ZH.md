# 室内起点前 10 米：定位 + S10 高层 SDK 导航试验

软件已单独实现并部署，默认只读；**尚未完成真机行走验收，不可直接宣称自主导航可用**。

最新真机 25 s 只读复测仍未通过实时放行：250 个状态采样中，按完整观测年龄要求有效的仅 49 个（19.6%），双雷达同时有效为 25 个。74 次配准被接受不等于能持续提供新鲜、双雷达可信的位置。IMU 排队和整帧排序开销已做针对性改进，但仍需解决剩余端到端延迟，并在正常站立状态重新验收。该结果是趴卧/低机身状态、现有 RL 服务仍运行时测得，不能直接推断暂停 RL 或站起来后一定通过。

本机目录：`E:\goai\goai_robotics_workspace\slam\indoor_navigation_v1`。
Orin 目录：`/home/wym/s10_indoor_navigation_v1`。

## 路线范围

按本次确认，取第一次录制 A 的起点沿已录轨迹累计前 10.0 m，不使用之前导出的室外 waypoint。保留实际弯曲路径及高度，不强制平地、不跨接捷径。生成 21 个独立测试点 I00–I20，朝向下一点；终点停车，不自动返回或继续室外路线。

- 起点约 `(0, 0, 0)`。
- 终点约 `(7.069, -1.074, -0.098)`，对应优化轨迹 offset 21.283 s；不要与原始录包接收时间混用。
- 高度变化约 9.8 cm；“室内”依据用户说明，仍需现场确认通道和地面。
- 原有 66 点、顺序、完整融合地图和旧定位工程均未修改。
- 地图保留路线周围 25 m 的 69,880 个地图点，室内检索库 71 项。裁剪只限制本次定位检索范围，不修改原地图。

![室内试验路线](indoor_route.png)

## 已实现的链路

`前/后雷达 + IMU + 可用的相机 IMU/VIO → 因果时钟与去畸变 → 固定地图匹配/室内重定位 → 顺序路线控制 → 保护网关 → S10 /NAV_CMD`

- 复用已有经过回放验证的匹配核心，以 `dependency_lock.json` 校验依赖。定位质量阈值未降低。
- 相机 VIO 仅作为受约束的短期运动增量；不拿 VIO 绝对位姿冒充地图定位。相机断流仍可尝试雷达定位。
- 点云与 IMU 使用 DDS 的实际接收时间，避免 Python 排队延迟污染时钟估计；无效/过期接收时间不放行。历史 IMU 缓冲与最新点云队列分离。每包仍检测时间跳变和年龄；稳定后的时钟漂移统计改为 20 Hz，减少高频重复计算。
- 实时点云先确定性采样再体素化，保留完整帧首尾时间，避免对整帧几十万个原始点排序导致观测过期。最终仍需通过相同几何质量门槛。这与已经预处理的离线回放输入不是完全相同的处理成本，必须单独做实时验收。
- SDK 消息包独立编译，不覆盖原 drdds。`/MOTION_INFO` 定义与在线类型哈希一致。NavCmd 的真机响应仍待验收，详见 `SDK_INTERFACE_EVIDENCE.md`。
- SDK 网关单独进程，分别连接传感器 domain 88 与原生 SDK domain 0，不转发整个 ROS 域。
- 上限 0.10 m/s、0.20 rad/s，横向速度固定为 0。每次启动需在 I00 附近进入，不能自动寻路接入起点。
- 双雷达均健康、前后感知新鲜、定位连续稳定 2 s、当前路段偏差合格、无近距障碍、SDK 状态和控制权合格，才能手动解锁。
- 定位重置、IMU 时钟 epoch 改变、感知/定位超时、操作员续期失效、SDK 异常、竞争发布者出现都会锁定零速。数据恢复不会自动续走；重新按 a 会重新验证当前路段，不跳过测试点。
- 实时避障不会滤掉人。跟随操作员过近也会触发停车。没有验证负障碍、台阶边缘或复杂避障能力；本次仅限现场清空的室内地面。

## 现在可以执行：只读观察

Windows 登录：

```powershell
ssh -4 -b 192.168.55.100 wym@192.168.55.1
```

Orin 中进入 Bash 并启动传感器；以下命令不启动运动控制、不录包：

```bash
bash
source /home/wym/s10_slam/env.sh
s10slam status
s10slam sensors-start
s10slam camera-start
cd /home/wym/s10_indoor_navigation_v1
bash start_monitor_tmux.sh
```

左窗定位，右窗 SDK 只读检查。`Ctrl+B` 再按方向键切窗，`Ctrl+B D` 仅脱离，进程仍在；各窗 `Ctrl+C` 结束。若不使用 tmux，可在两个终端分别运行 `bash run_localizer.sh` 和 `bash run_sdk.sh --seconds 0`。

默认自动在室内库检索，不把当前位置强制设为起点。只有物理位置已确认时才可使用 `bash run_localizer.sh --initial X Y Z YAW_DEG`，该初值仍需几何匹配确认。机器人趴着时机身地图 Z 会低于站立录制时，不能修改外参/强行拉高来绕过检查。

结束后，若没有其他录制任务使用这些传感器，可执行：

```bash
source /home/wym/s10_slam/env.sh
s10slam camera-stop
s10slam sensors-stop
```

## 真机试跑前必须解决的阻止条件

本次只读实机检查发现 `rl_deploy.service` 正在运行，`/NAV_CMD` 已有 2 个发布者。机器人反馈 state=0、gait=0，并非平地 RL 导航状态；也未检测到 `/dev/input/js0`。**程序不会抢占这些控制器，不会自动发站立/步态切换命令。**

现场应先将机器人安全趴下/进入官方安全状态，由可立即急停的操作员确认，再协调暂停现有第三方 RL 控制器；不要在机器人依赖它保持站立时直接杀进程。原机 planner/charge_manager 的控制入口尚需核对；应由 S10 管理端停止竞争的导航输出，保留 basic_server。仅停止 Orin 的 rl_deploy 不能解决那两个原机 NavCmd 发布者。

在接收端已可靠验证断流停车、遥控急停，并确认导航模式与平地步态后，重新检查：

```bash
cd /home/wym/s10_indoor_navigation_v1
bash run_sdk.sh --seconds 10
```

要求：`other_nav_publishers=0`、`rl_service_active=false`、`other_named_joint_publishers=[]`、`nav_subscribers>=1`；反馈 `state=17`、`gait=12290`（0x3002）、充电 idle。再现场确认定位与机身实际位置一致，连续稳定，通道清空。

软件已具备下列**人工现场试验入口**，目前不要直接执行：

```bash
# 仅在上面所有物理条件均真实完成、且操作员在场时使用这些确认参数。
bash run_sdk.sh --execute --seconds 0 \
  --onsite-confirmed --navigation-mode-confirmed --receiver-stop-verified
```

`a` 解锁；点按空格续期 0.4 s，需持续续期才有非零速度。只按一次空格不会持续行走；键盘长按的首次重复延迟可能超过 0.4 s，不应把键盘当物理急停。`s` 立即锁定软件零速，`q`/Ctrl+C 零速并退出。所有保护停车均需显式重新解锁。I20 到达后保持 COMPLETE，本进程不允许重新起跑。

**不能把上述三个确认参数当成安全条件的替代品。** 应先在现场监督下验证单小段低速响应及零速/断流/急停，再放行整段。SIGKILL、Orin 断电或网络故障时应用层无法保证停车，必须依赖接收端超时和独立物理急停。

## 验证范围与结果

- 15 项单元测试：默认无动作、竞争控制器、错误/缺失状态、过期/未来时间、位姿跳变、越界、高度异常、死手超时、恢复后不跳点、终点锁定、优化时钟与原时钟的稳态一致性、点云大小端/行填充/时间边界与格式错误。
- 理想运动学闭环：21 点全部依次通过，约 108 s 到达终点；只证明控制逻辑，不代表腿式机器人动力学。
- ROS localhost 隔离域 188/189：真实 NavCmd 序列化/订阅通过，定位中断超过门槛后的命令全部为零，停车保持锁定；未创建原生 domain 0 参与者。
- A/B 起点附近各 35 s 因果回放：有效定位时间占比约 95.1%/95.4%（含初始化等待）；各自前 10 m 的同图参考位置差 P95 为 9.8/9.4 cm，姿态差 P95 约 7.0/5.3°。这些是同图轨迹一致性，不是独立真值精度，不能承诺厘米级绝对定位。
- A 故障注入含前雷达断流、后雷达时间跳变、全部传感器中断、VIO 跳变、延迟包；全部中断后的超时样本没有有效定位或非零诊断速度。
- 真机只启动过本工程的传感器接收和定位只读检查，结束后停止了本轮启动的接收进程；旧控制服务没有动，原生运动命令发送数为 0。实时可用性仍以 `results/indoor_audit.json` 最新记录为准，不把离线回放结果替代为实机验收。

本机 WSL 重跑：

```bash
cd /mnt/e/goai/goai_robotics_workspace/slam/indoor_navigation_v1
python3 -m unittest discover -s tests -v
bash run_isolated_test.sh
source /mnt/e/goai/slam_local/env.sh
python3 scripts/replay_indoor.py --session A --output results/A_new_test
```

`live_logs/` 保存现场日志，`results/` 保存本机报告；`delivery_manifest.json` 校验软件交付。尚待：站立状态实时定位验收、原机控制权协调、NavCmd 真机响应及接收端停车验收、室内 10 m 真机闭环。
