# 官方运控 50 cm 地图导航试验

**更新：需要尚未验收的首次现场调试时，使用 [FIRST_TRIAL_ZH.md](FIRST_TRIAL_ZH.md) 和 `bash run_first_trial.sh --start`。** 下文是原正式网关操作，不能把它的 `acceptance.json` 前提套用到新增首测模式。

入口：Orin `/home/wym/s10_native_navigation_v1/run_trial_50cm.sh`。
不使用 GOAI 关节模型；速度通过 `.103` 网关交给原厂运控。

## 试验定义

- 保留原地图与 10 米路线；运行时截取其前 **0.50 m 水平折线路程**：`I00 → I01 → T050`，不是按计时开环前进，也不是从任意当前位置向前走。
- 地图起点约 `(0.0002, 0.0003, 0.0003)` m；终点约 `(0.4623, 0.1714, -0.0198)` m；起始朝向 `0.2344 rad`。这是地图坐标，不是房间的绝对方位。
- 前进速度上限 **0.20 m/s**、转向上限 **0.20 rad/s**，不倒走、不侧移。沿用 0.10 m/s² 起步加速与距离比例减速；因此不保证这段短路线会达到速度上限。原 10 米入口仍限 0.10 m/s。
- 地图降级仍限 0.04 m/s，有限预测 0.02 m/s，实测 VIO/IMU 接续 0.03 m/s；短避障延迟 0.02 m/s 且不转向。未延长任何观测有效期。
- ARM 前要求定位在起点 8 cm 内、朝向误差不超过 0.35 rad，并经过已有新鲜、静止观测窗口。当前段横向走廊 12 cm，垂向 10 cm。
- 到点半径 **5 cm**；先输出零速，再要求新鲜原始地图匹配、至少 3 个不同观测、连续 0.6 s 机身反馈静止，才显示 `COMPLETE`。这是估计位置判据，不是 5 cm 绝对精度证明。实际行程可能小于 50 cm，也可能受定位误差、滑移、制动而超出名义终点。
- 从 ARM 起最多 60 s；另有指令积分 0.50 m、累计指令转角 1.20 rad、地图位移/越界监测。这些预算不代替实测停车距离。
- 仅一次 ARM。到点、STOP、长时定位失效/障碍 HOLD、失联、控制权变化、超时等结束本次试验，不自动续走。程序仍可显示状态和接收 `LIE/QUIT`。不能用 `--resume-checkpoint` 接续短测。短时有效观测接续不会因单纯丢帧终止。

## 当前交付状态

软件实现与无运动测试完成，**不是实机 50 cm 行走完成**。部署核查时 `.103` 仍没有 `acceptance.json`，因此执行网关保持锁止。

0.20 m/s 涉及新网关代码哈希。有效验收还必须以真实证据覆盖该速度（`verified_forward_speed_mps >= 0.20`）、原厂接收端超时及停止距离，不能把旧 0.10 m/s 验收或模拟测试改名复用。没有创建通过记录、没有新增跳过参数。验收字段的真实性仍依赖实际测量；哈希校验不能证明物理安全。

## 执行步骤

以下终端保持开启，全部由操作者启动。场地须为平坦、无楼梯/边缘的隔离区域，路线及终点之外留有停车余量，机身急停可立即操作。原厂接口开启后不能依赖 G12 接管。不要停止正在支撑站姿的运控；先正常趴下再切换控制器。

### 1. Windows 登录 Orin，并预览（无运动、无网关连接）

```powershell
ssh orin-wifi
```

```bash
cd /home/wym/s10_native_navigation_v1
python3 scripts/verify_delivery.py
bash run_trial_50cm.sh --describe
```

应看到 `distance_m: 0.5`、`max_vx_mps: 0.2`。`--describe` 不启动 ROS 节点或网关。

### 2. 终端 A：在本体启动官方接口网关

从另一个 Orin SSH 终端登录本体，保留已核对的主机密钥：

```bash
ssh -o StrictHostKeyChecking=yes -o UserKnownHostsFile=/home/wym/s10_native_navigation_v1/body_known_hosts user@10.21.33.103
cd /home/user/s10_native_navigation_v1
python3 scripts/verify_delivery.py
python3 -c "from native_nav.acceptance import verify; from pathlib import Path; verify('acceptance.json', Path('/proc/sys/kernel/random/boot_id').read_text().strip()); print('ACCEPTANCE_OK')"
```

**上一步必须实际通过再继续**。当前文件缺失会直接报错；不能复制示例改成 true 代替验收。若需先只读检查，可运行 `bash run_gateway.sh --observe`，它只有状态心跳，不能导航。不要同时运行两个网关或两个客户端。

已通过真实验收、原有只读网关已正常退出时：

```bash
bash run_gateway.sh --execute --acceptance acceptance.json
```

### 3. 终端 B：Orin 隧道

```bash
cd /home/wym/s10_native_navigation_v1
bash run_tunnel.sh
```

该终端没有持续文字输出属正常；保持连接。

### 4. 终端 C：Orin 传感器和定位

```bash
bash
source /home/wym/s10_slam/env.sh
s10slam sensors-start
s10slam camera-start
cd /home/wym/s10_native_navigation_v1
bash run_localizer.sh
```

已运行的定位实例不要重复启动。这里不启动运控，也不自动站立。

### 5. 终端 D：Orin 短测入口

```bash
cd /home/wym/s10_native_navigation_v1
bash run_trial_50cm.sh --execute
```

默认不加 `--execute` 为只读。加上后也不会自动动作；在此终端逐条输入，每条回车并检查 ACK 和真实反馈，**不要整段粘贴动作序列**：

1. `SDK_OFF`：仅在网关确认已趴下且静止时关闭 SDK。
2. `MODE_NAV`：确认使用模式变为 1。
3. `STAND`：请求原厂站立，等 `state=17`、反馈静止。原厂首次起立可能含约 14 s 自动标零，不能盲目重复发送。
4. `FLAT`：等 `gait=12290`（`0x3002`）、`mode=1`，定位稳定且 `ready=true`。
5. `ARM`：开始这一次 50 cm 试验。

如果已处于对应模式/站姿，跳过已经完成的姿态步骤，不重复起立。若状态为 `0`（Idle），当前导航网关不会把它伪装成 `4`（已确认趴下），`SDK_OFF/MODE_NAV/STAND` 会拒绝；此时需要解决固件状态/原厂姿态衔接，不能靠连续重试或删除门控进入。该问题与本次路线长度功能分开。

### 6. 停止与收尾

- 随时输入 `STOP` 回车；正常到点显示 `COMPLETE`，并解除路线运动许可。
- 确认实际静止后，输入 `LIE` 回车，确认已趴下，再输入 `QUIT`。
- Ctrl+C/SIGHUP 会尽力请求零速并退出；网络或进程异常时不能保证停车命令到达。若机器没有按预期停下，使用现场独立机身急停，不等 SSH 恢复。
- 同一次进程不能再次 ARM；再次试验需先回到地图起点、退出重开并显式 ARM。

## 日志与无硬件验证

Orin 日志：`/home/wym/s10_native_navigation_v1/live_logs/native-trial50.jsonl`（轮转限额）。进度单独记录为 `trial50-progress.json`，不覆盖原 10 米检查点、不恢复运动许可。

```bash
cd /home/wym/s10_native_navigation_v1
PYTHONPATH=/home/wym/s10_slam/venv/lib/python3.12/site-packages python3 scripts/test_all.py
```

测试网关仅 OFFLINE/模拟 loopback，不创建机器人 UDP 接口；模拟不能代替真实控制模式或停车验收。
