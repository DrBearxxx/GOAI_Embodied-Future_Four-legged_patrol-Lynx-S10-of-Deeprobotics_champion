# S10 原厂运控 + 现有地图导航（室内 10 米）

2026-09-20 更新：当前首测使用 `SDK_OFF → STAND → MODE_NAV → FLAT → STATUS → ARM`；允许稳定定位下平移临时 50 cm 起点，回执区分网关已下发与实际反馈已确认。完整说明以 [FIRST_TRIAL_ZH.md](FIRST_TRIAL_ZH.md) 为准，以下正式导航流程不适用于本次首测的姿态准备顺序。

**首次现场短测的新入口**：`bash run_first_trial.sh --start`，见 [FIRST_TRIAL_ZH.md](FIRST_TRIAL_ZH.md)。此单次 50 cm / 0.20 m/s 模式不以已完成的 `acceptance.json` 为前提，状态如实记为 `UNVERIFIED`；具有本体单次预算及零速伴随进程，不开放 10 米路线或自动续走。以下验收文件要求只适用于原正式导航 `--execute` 入口。

新增独立 **50 cm、上限 0.20 m/s** 入口 `run_trial_50cm.sh`，默认只读、单次 ARM；操作步骤与当前验收限制见 [TRIAL_50CM_ZH.md](TRIAL_50CM_ZH.md)。下文 10 米路线仍限 0.10 m/s。网关/协议允许上限更新为 0.20 m/s，因此网关代码哈希变化，旧验收不能复用；新验收须以实际证据覆盖 `verified_forward_speed_mps >= 0.20`。

本目录是独立执行后端，不覆盖 GOAI 控制器、冻结地图、外参和路线。路线仍为第一次录制起点的 I00–I20（21 点，约 10 米），按原顺序行走、朝向下一个点，终点停车，不自动原路返回。仅平地试验；未实现楼梯、负障碍保护或全场路线运行。

## 实现与边界

- Orin：使用独立 V3 鲁棒定位入口及 `resilient_navigation_v1` 更新，继承冻结 V2 的地图匹配、IMU 冗余、避障和路线走廊。生产路线入口使用 `ResilientRoute`；旧 `NativeRoute` 保留用于基线回归。地图与旧 V2 文件不变。
- `.103`：独立 Python 网关，ASDU 真实轴 `Type=0x00100001 / Command=0x00110002`，X/Y 为 m/s，Yaw 为 rad/s。不是摇杆比例指令 `0x00100002`，不是关节 SDK。
- 使用模式 `Mode=1`、运动状态 `17`、平地导航步态 `0x3002` 必须反馈确认。站立与模式切换均需操作者显式输入，不运行自动起立/前进序列。不实现软急停状态 -2 下发（文档中该状态只读）。
- 原厂 UDP 只在 `.103` 本机使用；Orin 通过 SSH 隧道连网关的 `127.0.0.1:18891`。不把自建控制 TCP 端口暴露到局域网，不保存登录密码。
- 网关发送速度 10 Hz；导航计算 25 Hz；接收端一次性挑战票据有效 150 ms，速度有效期从**票据签发时刻**起最多 250 ms。使用各自主机单调时间和往返期限，不混用跨机 monotonic 时钟。
- 网关票据过期/重放、断连、机身状态过期、模式/步态不符及其他控制器仍撤销运动许可，不自动重新 ARM。感知短缺则优先冗余接续/降速，证据不足才 HOLD；同一许可内 HOLD 的恢复见下节。显式 STOP 后数据恢复不会自动行走。
- 网关自身进程被杀或 `.103` 断电时，它无法再发零速。必须实测原厂接收端超时停车及独立硬件急停；SSH、G12、Ctrl+C 都不能冒充独立硬急停。
- 2026-09-19 群公告说开启后遥控器不可使用。未实测推翻前，不依赖 G12 接管。关闭 TLS 只适用于隔离机器人内网；本实现不会自动修改该配置、停止厂家服务或重启机器人。
- 指南示例是秒级时间戳；2026-09-19 实机通知携带毫秒时间戳，状态类型为 `0x00300064/0x00300001`，且 `Sleep` 为整数 0/1。接收端显式兼容两种状态类型和时间格式；毫秒通知还检查源时间递增、状态时效，内核到达时间防积压。报文时间仍不证明控制器内部采样时效，须实测。ROS 图和进程检查不能发现所有外部 UDP 控制者，必须保证独占控制网络。

## 连续跟随更新（2026-09-19 晚）

目标不是“缺一帧就停车”，而是先利用剩余有效观测继续低速走；没有可靠运动/避障证据时才停车。工程预算不等于已标定协方差或实机精度保证。

- 地图修正计数变化不再直接撤销许可；仍检查实际位姿跳变、时效、走廊和控制权。
- 连续且同一 epoch 的 VIO 与实测陀螺增量一致时，用它们估计短时身体速度，避免把地图匹配噪声当作实际速度。小幅地图校正采用有界融合（增益 0.60；输入校正不超过 0.24 m / 0.15 rad）。保留原始地图匹配位姿、观测时间和融合残差，残差占用路线走廊余量；拒绝的异常匹配不能刷新定位时效。没有有效 VIO/IMU 时不会假装融合成功。
- 地图匹配间隔超过 0.55 s 时，新增 `ODOM_BRIDGE`：必须有真实、同一 epoch、无长缺口的 VIO 与连续 IMU，旋转一致；不是只沿用上一速度。允许地图观测年龄最多 1.20 s，累计 VIO 路程最多 0.15 m，上限 0.03 m/s、0.06 rad/s。VIO/IMU 端点仍须在 80 ms 内，超过 40 ms 禁止转向。桥接期间不能确认 waypoint 到达。
- 坏包/重复包隔离，上一有效包只在其原有效期内可用，不重打时间戳。同一地图中尚未确认的新候选不会立刻抹掉仍有效的旧锚点；真正地图 epoch 改变仍需重新验证。
- HOLD 只发送新鲜零速度，保留目标点和已有许可。停止包络内（0.20 m / 0.25 rad）、机身反馈静止、双侧避障新鲜、至少 2 秒且至少 6 个不同的真实地图观测稳定后，才从零速缓慢续走。普通扫描间隔中的 DEGRADED 不反复清空恢复窗口。持续的大位姿偏差不能靠等待自动放行。
- 操作者 STOP、网关断连/许可丢失、定位进程 run-id 改变、地图/时钟身份改变、控制权冲突仍需重新显式 ARM；后台重连仅恢复通信，不重放旧动作。不要把“任务还活着”理解为“始终允许走”。
- 路线日志跨运行限制为 4 × 8 MiB；日志写失败不退出循环。执行模式保存目标点检查点；`--resume-checkpoint` 可恢复相同地图/路线的进度，但不恢复 ARM。默认重启仍从 I00 开始；只读检查不覆盖执行进度。

## 基础鲁棒性更新（2026-09-19）

`run_localizer.sh` 已默认进入 `run_robust_localizer.sh`，无需修改路线或地图。保持 `goai.localization.continuity.v2` 消息兼容，附带 `continuity_implementation=native_v3`。

- IMU 最新映射时间轻微超前时，查找已经收到、且不晚于当前时间的真实历史采样，避免误丢整条 IMU 流；不把未来样本改成当前时间。
- 时钟偏移以最高 200 ppm 小步跟随慢漂移，单次修正不超过 20 微秒；重复、倒退、突跳、过期仍拒绝，历史时间戳不重写。
- 工作进程异常退出自动重建。匹配循环和 ROS 回调循环有独立存活检测：首次启动容忍 45 秒，正常循环无心跳超过 5 秒触发重建；这些是**进程恢复**时限，不是运动允许继续的时限。运动证据早在下述短期限内过期。
- 传感器缺数据不等于进程死机；处理循环仍活着就继续等数据，不因断流反复重启。单传感器异常隔离、有限 IMU 预测及零输出保护保留。
- 定位重启采用 2–30 秒退避，清理自身子进程。只重启非驱动定位进程，绝不自动起立、重启运控、ARM 或发送轴速度。重启后 run-id 改变，路线锁止；需新鲜观测重新建立定位，再显式 ARM。同一路线进程保留目标点，重启路线程序不自动恢复旧进度。
- 三类新增定位日志合计上限约 68 MiB；写日志失败降级并重试，不直接终止处理循环。此上限**不包含**传感器驱动日志、旧日志、录包、ROS 系统日志或用户开启的 profiling 文件；磁盘完全耗尽仍可能影响启动和系统组件，不能保证整个系统不受影响。

长时间运行建议在 tmux 内启动定位，防止 SSH 断开连带结束前台程序：

```bash
tmux new-session -s s10-localization 'bash /home/wym/s10_native_navigation_v1/run_localizer.sh'
```

Ctrl+B、D 仅分离终端；重新连接用 `tmux attach -t s10-localization`。停止定位用该会话内 Ctrl+C。不自动启动导航行走。未安装开机自启或系统级 supervisor 保护；Orin 关机、整机资源耗尽、supervisor 本身被杀不在本轮自动恢复覆盖范围。

## 掉帧与停车预算保持不变

正常上限 0.10 m/s、0.20 rad/s；短时单雷达/几何延迟降到 0.04 m/s；有连续 IMU 的有限恒速预测降到 0.02 m/s。纯预测地图年龄仍最多 0.55 s；只有上述严格实测里程计接续可以到 1.20 s。IMU 最长 0.08 s、历史双侧无障碍证据最长 0.40 s 没有放宽。短 IMU/避障延迟禁止转向，预测和里程计接续不确认到点。超界时保留任务并按 HOLD/许可撤销的类型处理，而非退出程序。

这些是工程试验预算，不是已标定精度或制动距离保证。真实人员仍属于避障对象。

## 安装位置

- 本机：`E:/goai/goai_robotics_workspace/slam/native_navigation_v1`
- Orin：`/home/wym/s10_native_navigation_v1`
- `.103`：`/home/user/s10_native_navigation_v1`
- 冻结依赖：`/home/wym/s10_indoor_navigation_v1` 和 `/home/wym/s10_route_navigation_v1`

两端代码可用 `python3 scripts/verify_delivery.py` 校验；Orin 另执行 `python3 -c 'from native_nav.bootstrap import verify; print(verify())'`。旧依赖任何变化都会拒绝运行，不重算旧清单掩盖变化。

## 最简操作流程

先不启动运动。现有 GOAI、原 `rl_deploy`、旧 SDK 网关、厂家 planner/charge_manager 等速度控制源不可与本后端同时控制。机器人若仍靠某策略站立，先正常趴下，再停止旧控制器；不要直接杀支撑中的关节策略。

### 1. `.103` 启动只读网关

从 Orin 执行 `ssh user@10.21.33.103`，使用已核对的主机密钥。以下命令发**状态心跳**，不发运动命令；但公告未明确遥控器失效的触发环节，故仍需在安全趴下时运行。

```bash
cd /home/user/s10_native_navigation_v1
tmux new-session -s s10-native-gateway 'bash run_gateway.sh --observe'
```

Ctrl+B，再按 D 可退出 tmux 界面而保留网关。默认 `bash run_gateway.sh` 是纯 OFFLINE，连机器人心跳也不发送。

### 2. Orin 建立隧道

另开 Orin 终端，保持以下命令运行（如需密码，交互输入，不写进脚本）：

```bash
cd /home/wym/s10_native_navigation_v1
bash run_tunnel.sh
```

脚本固定已核对的本体 SSH 公钥。主机密钥变化时会拒绝连接；不要用 `StrictHostKeyChecking=no` 绕过。

### 3. Orin 启动传感器、定位、只读路线检查

```bash
bash
source /home/wym/s10_slam/env.sh
s10slam sensors-start
s10slam camera-start
cd /home/wym/s10_native_navigation_v1
bash run_localizer.sh
```

另一 Orin 终端：

```bash
cd /home/wym/s10_native_navigation_v1
python3 scripts/status_client.py --seconds 5
bash run_route.sh
```

一个网关只接一个客户端；`status_client.py` 结束后再启动路线。只读网关会显示 `FACTORY_GATEWAY_NOT_EXECUTING`，这是预期，不能因此把执行保护关掉。实际操作前还要完成下述实机停止验收。

### 4. 完成实机安全验收后才启用执行

单独的原地姿态复测入口为 `python3 scripts/posture_trial.py --execute`（在 `.103`，先退出只读网关）。它只在当前常规模式 0、连续新鲜且静止的低机身 Idle/趴下状态下，执行一次 STAND、20 秒站姿观察、一次 LIE；不发任何轴指令，不切 SDK/模式/步态，不 ARM、不修改导航验收记录。默认不加 `--execute` 只观察。实机首次 STAND 会由原厂自动触发约 14 秒标零，再起立；不是本脚本直接下发标零命令，不能假定站立请求超时后不会继续动作。脚本现等待最多 40 秒，不重发 STAND；如已站立，可用 `--execute --finish-standing --hold-seconds 5` 做采样后趴下收尾，不再发送 STAND。状态失效或异常时不会盲目发趴下，更不会关闭支撑中的运控；须查看最终实机反馈。此测试需要现场已留出起立空间且有独立急停手段，但不把“急停按钮存在”当作急停功能实测通过。`.103` 的进程检查受容器可见性限制，不能凭它宣称控制来源已完全排除。

`acceptance.example.json` 全部默认未验证，**不能直接改成 true 来放行**。需要记录独立硬急停、Orin 断连、网关强制退出、速度单位/方向实测，停止距离不大于 0.10 m，原厂接收端指令失效不晚于 0.30 s；这是当前低速试验的验收上限，不是已测出的设备性能。首次验收应由现场受控调试完成；本包不提供绕过接收端验收的非零速度入口。

将实测记录保存在 `.103`，填写 `acceptance.json` 的证据路径及 SHA256、操作员、实测数值、当前 `.103` boot_id，以及 `python3 -c 'from native_nav.acceptance import code_hash; print(code_hash())'` 得到的代码哈希。重启 `.103` 或修改网关后，旧记录不能直接复用，需要复核。程序校验记录的一致性，不能自动证明记录内容真实。

停止只读网关后，在同一位置启动：

```bash
bash run_gateway.sh --execute --acceptance acceptance.json
```

Orin 保留定位和隧道，重启路线终端：

```bash
bash run_route.sh --execute
```

路线终端按需**逐条**输入，下发结果需看反馈，失败不能继续盲发：

1. `SDK_OFF`：仅在原厂反馈已经趴下时允许，不强行退出未知 SDK 支撑状态。
2. `MODE_NAV`：保持趴下，切导航使用模式。
3. `STAND`：使用原厂策略站立。ASDU 站立完成后按文档自动进入状态 17。
4. `FLAT`：确认已站稳且速度低于 0.03 m/s，切 `0x3002`，等反馈一致。
5. 定位在 I00 附近（0.25 m 内）、连续 2 秒新鲜且 `ready=true` 后输入 `ARM`，此时开始路线行走。
6. `STOP` 锁止停车；确认实际静止后 `LIE` 趴下，最后 `QUIT`。STOP 不是硬急停。

实际测试先走 1–2 米就 STOP，确认方向、响应和停止，再进行整个 10 米室内段。同一进程重新 ARM 保留目标点；网关重连无需退出路线程序，但须重新显式 ARM。若需恢复上次执行的目标点，运行 `bash run_route.sh --execute --resume-checkpoint`，地图/路线校验不符则拒绝；该参数从不自动开始行走。

## 无硬件测试

在 Orin（不启动传感器、运控或网关服务）：

```bash
cd /home/wym/s10_native_navigation_v1
PYTHONPATH=/home/wym/s10_slam/venv/lib/python3.12/site-packages python3 -m unittest discover -s tests -v
```

TCP 集成测试只启动 `--simulation` 或 OFFLINE 子进程，监听随机 loopback 端口，**不创建机器人 UDP 套接字**。`--simulation` 与实机 `--execute` 互斥；实机路线拒绝模拟网关反馈。理想运动学全路线到达不是实际定位精度或动力学测试。

## 接口依据与本轮范围

依据 2026-09-19 群发布的《软件开发指南》MD5 `47d96262519d045f7f9fb7ec21e8f3cb` 与示例 MD5 `b6aed078d7d2c0aac2c10ed79a56635a`。未直接执行示例中的自动站立/楼梯/前进序列。

本包实现软件后端与部署入口，不宣称原厂接收端超时停车、G12 行为、实机定位精度和路线闭环已验收。
