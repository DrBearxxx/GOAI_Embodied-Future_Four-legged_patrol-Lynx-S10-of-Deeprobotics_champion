# GOAI 室内 10 米导航

本入口取代旧高层 SDK `/NAV_CMD` 网关；GOAI 默认模型仍负责关节，导航仅向 `/wym/goai/nav_cmd_vel` 发送 `base_link` 的 TwistStamped。原始地图、66 点室外路线、室内 I00–I20 的顺序未改动。仅限第一次录制起点沿轨迹前 10 米，走到终点停止，不自动返回。

末尾关节故障由用户确认是主动关闭，本轮不当作随机故障，也没有删除保护检查。新路线适配通过软件/隔离测试不等于真实环境闭环已验收。

## 先做只读定位验收

Windows 登录 `ssh -4 -b 192.168.55.100 wym@192.168.55.1`。Orin 每个终端先执行 `bash`。

终端 1：启动传感器和定位（不录包、不发运动指令）。

```bash
source /home/wym/s10_slam/env.sh
s10slam sensors-start
s10slam camera-start
cd /home/wym/s10_indoor_navigation_v1
bash run_goai_localizer.sh
```

终端 2：观察一分钟，不创建速度发布者。

```bash
cd /home/wym/s10_indoor_navigation_v1
bash run_goai_route.sh --seconds 60
python3 scripts/audit_goai_route.py
```

报告 `localization_valid_fraction` 看定位与双雷达时效；`gate_ready_fraction` 还包含 GOAI 状态、控制权及当前位置与路线的匹配。GOAI 未在健康的手动策略状态时，后者为 0 是正常拦截。需要在正常站立姿态再做只读复测，现场确认位姿确实落在起点附近、朝向正确。不要随意输入初始位姿或挪动地图来让检查通过。

历史实时可用性未通过，仍需现场复测。V2 已增加有界短时降级，详见 `ROBUSTNESS_V2.md`；不再将每次单路掉帧等同于整机停车。旧严格时效比例保留作为对照，新增 `v2_estimate_available_fraction` 仅表示估计连续性。启动必须连续 2 秒新鲜双侧覆盖和 TRACKING，运行中可以在预算内限速降级；超限或真正失去避障/定位依据仍锁止停车。相机可缺席；单路时钟隔离不直接清空其他可用传感器。报告的 `ready_for_supervised_trial` 仍是保守筛查，不是精度或物理安全保证。

## 现场确认后试跑

先确认安全趴稳、起点附近、无台阶边缘/孔洞/人员或杂物、G12 与独立物理急停可用。按已经验证的流程启动 GOAI：本次开机重新核对原生 GUID，`SDK-READY` → 摇杆回中 → C 站立 → 站稳后 A 手动策略。本文不复用旧 GUID，也不自动操作 SDK 模式或站立。

保留终端 1 的定位。**退出 `nav_zero.py` 和旧 `run_sdk.sh` 网关**；同一专用速度话题只能有一个发布者。不要在 GOAI 正支撑机器人时杀掉其运控进程。

终端 2：

```bash
cd /home/wym/s10_indoor_navigation_v1
bash run_goai_route.sh --execute --onsite-confirmed
```

终端 3（GOAI 运控 tmux 保持原样，另开此终端）：

```bash
cd /home/wym/s10_goai_ws
source env.sh
python3 nav_guard.py --allow-nonzero
```

输入本体 user 的密码，等待 `G12=LIVE`。然后严格按顺序：

1. 终端 2 显示 `ROUTE_READY` / `ready: true`，输入 `ROUTE-READY`。
2. 终端 3 输入 `NAV-READY`。路线准备有效期 30 秒，超时需重新确认。
3. 出现 `PERMIT READY` 后，松开 C 至少 1 秒，10 秒内按 C。**此时可能真实开始转向/行走**；`FOLLOW` 表示跟随中。

上限前进 0.10 m/s、转向 0.20 rad/s，无横移/倒退，朝向当前目标点；大角度先原地转向。启动必须在 I00 约 0.25 米内；后续横向偏差超过 0.45 米、高度偏差超过 0.22 米则停车。不自动抄近路、跳点或寻路返回路线。

## 停止、接管、继续

- A：退出导航、回到手动；明显拨动摇杆也可接管。B 趴下，D 阻尼。软件键不能替代物理急停。
- 任一操作终端输入 `STOP` 取消本轮路线/许可。路线端会先发一次零速，然后在导航仍被选中时撤回速度流，让 GOAI 独立 0.25 秒输入超时锁定零目标；现场停止距离仍需验证，不声称立即物理静止。
- 短时掉帧可限速继续，恢复后经过限速缓冲再回正常；但一旦因超时、重定位、障碍、竞争发布者、许可丢失或程序退出触发停车，**恢复数据不会自动续走**。先排除原因，回中按 A，再按 `ROUTE-READY → NAV-READY → C` 重新授权。
- 同一进程内继续会保留当前目标点，不跳过剩余点。终点 COMPLETE 后本进程不再起跑。重启路线进程回到 I00，不自动接着上次进度。
- 结束先 B，确认安全趴稳/disarmed，再退出辅助程序与运控。当前如已触发故障，不能依靠 B 越过故障强制趴下，需现场安全处置。

## 软件核验

```bash
python3 scripts/verify_goai_overlay.py
```

使用独立 `goai_overlay_manifest.json` 核验增量、依赖、地图及原路线。原 `delivery_manifest.json` 的历史不一致已保留，没有用重算旧清单掩盖它。旧定位单进程文件和现场检查文件未被覆盖；新入口明确使用多进程 `localize_mp.py`。

`bash run_goai_isolated_test.sh` 只在 localhost 的 187/188 域运行合成关节/定位，使用真实 GOAI 二进制和真实 ROS 序列化，不连接真实机器人控制域。它不能代替现场接收端超时、急停与行走精度验收。
