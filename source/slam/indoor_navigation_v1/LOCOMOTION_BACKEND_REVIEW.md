# 运控后端核查：2026-09-13

本文件是只读核查记录与后续接入建议，不表示已切换后端或完成真机行走。现有 `sdk_gate.py` 仍为原生 `/NAV_CMD` 后端，不可把它直接用于下面的 GOAI 关节运控链路。

## 当前服务是什么

`rl_deploy.service` 的 User 为 `xxx`，WorkingDirectory 为 `/home/xxx/goai_embodied_future_material`，经 `/home/xxx/tremor_test/rl_supervise.sh` 和 `start_rl.sh` 启动 `install/s10_sdk_deploy/lib/s10_sdk_deploy/rl_deploy`。

核查对应源码：main.cpp 选择 `RemoteCommandType::kDDS`；`dds_command_interface.hpp` 订阅 `/STEER`、`/GAMEPAD_KEY`；`rl_control_state.hpp` 指向源码树的 `policy/policy.onnx`；`s10_policy_runner.hpp` 把机身 IMU、关节状态、前次动作及速度目标组织为策略观测，计算腿部关节位置和轮速度目标。它是低层 RL 运控部署程序，不是地图定位/导航算法，也不能仅凭可执行文件名确定模型训练来源。

当前 xxx 代码存在默认姿态选择、软启动和诊断发布等改动。启动脚本设置 `S10_RL_DEFAULT_POSE=metadata`、`S10_RL_SOFTSTART_STEPS=100`。只检查了与运控有关的环境变量，没有导出完整环境文件。

## 模型身份（SHA-256 文件校验，不等于权重差异分析）

| 文件 | SHA-256 |
| --- | --- |
| 本机 `E:/goai/goai_embodied_future_material/src/S10_sdk_deploy/policy/policy.onnx` | `92db62c118c4ebad3da8bfedb89691f8caae8803966b472b4ce541e1a32f2d0d` |
| Orin `/home/wym/goai_embodied_future_material/src/S10_sdk_deploy/policy/policy.onnx` | `92db62c118c4ebad3da8bfedb89691f8caae8803966b472b4ce541e1a32f2d0d` |
| 当前服务对应 xxx 源码树 `policy/policy.onnx` | `6bc9bf452386d273f8e5e75caba3a8d0086709e678a5e846a176e68d68dab37e` |
| 先前独立 WSL blind 部署 `s10_blind_ws/src/s10_blind_deploy/policy/actor.onnx` | `26e8a8443011cd66b857ec032cdc54eb8f1f02e3904922f99df7f061c4da1e6f` |

因此不能声称当前 xxx 模型与 GOAI 默认文件相同，也未证明两者只差 metadata 或确实有权重差异。默认文件在 wym 中完整保留。未启动任何模型来验证其运动效果。

## 建议后端分层

固定地图定位/重定位 → 顺序路线跟随（vx、vy、wz） → 安全仲裁与速度接收看门狗 → 唯一 GOAI 默认 RL 运控实例 → `/JOINTS_CMD` → 机器人驱动。

导航不计算关节动作，不改模型权重，不用相机 VIO 绝对坐标代替地图定位。原生 `/NAV_CMD` 驱动原机运控与 GOAI SDK 关节控制是不同的执行后端，不能同时放行。原先 Guard 中 `rl_service_active` 被视为冲突，只适用于原生 NAV_CMD 后端；GOAI 后端必须改为验证指定运控实例身份、健康及唯一关节发布权，而不是直接删除所有权检查。

核查默认和 xxx 当前代码，两者在策略输入前均执行：`vx=axis_x*1.5`、`vy=axis_y*0.5`、`wz=axis_yaw*0.6`。这是策略期望速度映射，不保证实测速度完全跟踪。不能把 m/s、rad/s 的导航指令直接当归一化 `/STEER` 轴值发布。

优先在 wym 独立副本中增加专用导航速度入口（明确 SI 单位），在运控内部做导航/手动单源仲裁，保留原手柄趴下/阻尼的优先权。不要在已有原机 `/STEER` 上增加竞争发布者。当前 DDS 接口只是保存最新轴值，`GetUserCommand()` 直接返回缓存；未见速度输入年龄看门狗，不能假定导航发布者断流后缓存自动清零。需补接收端超时、非有限值/过期消息拒绝、显式重新解锁，并隔离测试后再进行低速现场验收。

## 当前物理和软件状态

用户确认安全趴下且能立即急停后，本轮已临时停止 `rl_deploy.service`。末次复核 MainPID=0、ActiveState=inactive、UnitFileState=enabled；未禁用开机启动，未修改 xxx 或 ysc 的代码/模型，未恢复运控，未发送运动或站立指令。接收/定位测试所创建的前后雷达和相机进程均已结束。

新多进程定位 60 s 静止只读测试 `1789302010625753672`：有效定位占比 92.24%；5 s 后 94.45%，同时满足双雷达有效及前后感知新鲜的比例 72.25%。这不是完整导航放行率，也不是绝对精度；仍不足以证明可连续安全走完 10 m。详见 `results/latency_v2/health-1789302010625753672-summary.json`。实验实现尚未替换已发布的默认定位启动入口或重新封装交付包。

后续需确认采用本机默认模型还是 xxx 当前模型，再实现相应独立后端；不得用当前原生 SDK 的 state=17、gait=0x3002 检查冒充 SDK 关节模式的就绪判断。
