# S10 固定地图定位与路线跟随验证 V1

## 当前交付边界

已实现路线编译、因果式多传感器定位/重定位、故障回放、路线跟随影子输出，并部署到 Orin。
**这是可以运行的影子验证版本，不是已通过验收的无人自主导航版本。** 本包没有运动 SDK、`cmd_vel` 发布器、TF 广播器，不切换 S10 步态、不驱动电机。

完整回放结果和未通过项目见 `VALIDATION_REPORT.md`（由实际结果生成）。建图使用过 A/B 两段数据，因此与地图参考轨迹的差异只能说明自洽性，不能当作独立定位精度或成功保证。

本机目录：`E:\goai\goai_robotics_workspace\slam\route_navigation_v1`。
Orin 目录：`/home/wym/s10_route_navigation_v1`。现有 `ysc`、`wym/s10_slam`、原始数据和地图没有被替换。

## 路线按你的确认处理

- 原 JSON 的 **66 个 waypoint，ID、XYZ、顺序全部原样保留**，没有删点、合点或重新排序。
- 每点 yaw 指向下一点；最后一点没有下一点，沿用最后一段方向。
- 不合并 B31/B32，不把末点改成 A426；终点仍是你导出的最后一点。
- `assets/route.json` 是编译路线；`assets/route_samples.csv` 是约 0.2 m 间隔的折线采样，总长度约 271.12 m。
- XYZ 包含楼层高度，路线跟随不会单凭 XY 就把另一层的点判成到达。
- 较长连线的中间走廊、楼梯可通行性仍需要实地核验。这不否定你确认的点位和顺序。8 段自动几何提示会触发 `HOLD_ROUTE_VALIDATION`，不会自动挪动你的点。
- 首点 B01 不在录制起点附近。启动后必须人工到首点附近，程序不会跳到距离最近的后续点，也不会凭空规划一条接入路线。

## 已实现的数据链路

1. 每路独立检查原始时间戳、接收时间、重复、回退、跳变和积压。仅用此前到达的 IMU 估计设备时钟偏移；跳变进入新时钟段并重新预热。不使用离线全包时钟拟合给在线程序提供先验。
2. 前后雷达/相机 IMU 经各自外参变换到机身坐标。连续陀螺仪用于点云旋转去畸变；缺少足够连续 IMU 的扫描拒绝使用，不跨长缺失插值。
3. VIO 只提供通过跳变和角速度检查的局部平移预测，不作为地图绝对位置。考虑到 VIO 发布晚于雷达，允许用已收到的最后两帧作不超过 100 ms 的有界外推，超过即停用该预测。
4. 前后雷达分别时间校验；时间相近时去到同一时刻，进行联合 GICP，并分别验证各路残差。不合格时尝试当前单雷达。
5. 冻结 20 cm 地图、3 级 GICP、保留点校验、重叠率/残差/退化/位移检查。有效平面单独建索引；对只有平面支持不满足的高重叠扫描，额外要求两组不相交点分别配准且结果小于 8 cm/1.5°，不能只凭一次低残差放行。这仍不能完全排除重复环境中的错误匹配。
6. 丢失后先做有界局部重捕获，再从固定候选库检索位置和偏航，多候选几何验证并拒绝接近得分的矛盾解；连续 3 次确认后才标记 `valid`。重力可由静止加速度估计，或从最近地图姿态经连续陀螺仪作最多 20 s 的检索姿态传播，不能跨 IMU 断档。
7. 深度图进入近场正障碍物检查。相机倒装外参保留；当前沿用原建图标定，平移仍是名义值，`calibration_verified=false`。深度不作为主定位校正源，RGB 不参与在线定位。
8. 独立定时看门狗使失效输出归零。单路雷达正常时允许定位继续；前向感知缺失、双雷达长期无有效约束、地图匹配失败、越出路线走廊、重定位后路线重新关联未确认时，不给出行走建议。

不是完整惯导 ESKF/因子图；不输出经过标定的协方差。点云平移去畸变使用历史速度近似。当前没有低层运动控制、完整负障碍物/楼梯边缘检测或独立连续 `odom -> base_link` 发布实现，不能直接接现有自动导航控制器当成已验收定位源。

## Orin 操作

先保持机器人由原有遥控器/S10 自带策略控制。以下只是传感器和影子验证，不接管行走。

在已经配置好的 Windows SSH 终端连接 Orin（如本机 USB 地址不存在，先检查连接，不要强制绑定不存在的地址）：

```powershell
ssh -b 192.168.55.100 wym@192.168.55.1
```

在 Orin 上启动现有传感器；已经运行则只执行 `status/check`，不要重复启动驱动：

```bash
source /home/wym/s10_slam/env.sh
s10slam status
s10slam sensors-start
s10slam camera-start
s10slam check
```

再启动影子定位（建议单独 tmux 窗口）。不提供位置时会尝试全局重定位，建议先站稳收集稳定 IMU：

```bash
tmux new-session -s s10-route-shadow 'bash /home/wym/s10_route_navigation_v1/run_orin_shadow.sh'
```

仅当机器人确实在这次地图的原录制起点附近时，可以用人工近似初值辅助，参数是地图 XYZ（米）和 yaw（度）：

```bash
bash /home/wym/s10_route_navigation_v1/run_orin_shadow.sh --initial 0 0 0 0
```

不要在任意位置照抄该初值；程序的几何检查也不是错误初值的绝对保险。

另一终端检查：

```bash
source /home/wym/s10_slam/env.sh
ros2 topic echo /wym/route_nav/health
```

主要输出：

| 话题 | 内容 |
| --- | --- |
| `/wym/route_nav/health` | 状态、valid、测量年龄、各路时钟状态 |
| `/wym/route_nav/diagnostic_pose` | 仅在新鲜且确认通过时发布的地图位姿 |
| `/wym/route_nav/route` | 保留用户顺序、已更新 yaw 的路线 |
| `/wym/route_nav/shadow_command` | 跟随建议/停车原因；`motion_authorized` 始终为 false |

`Ctrl+C` 退出验证；tmux 的 `Ctrl+B`、再按 `D` 只是退出观察，程序仍在运行。日志在 `/home/wym/s10_route_navigation_v1/live_logs/`。

在验证过程中由你人工沿原路线行走，查看掉帧后是否停止发布有效定位、恢复到的楼层/走廊是否正确。不要把影子速度话题转换后直接接电机。`HOLD_REASSOCIATION` 当前是锁存保护；需要保留进度的操作员重新关联界面尚未实现，不能靠自动跳过 waypoint 解除。

## 本机复现

下列命令在 PowerShell 执行。脚本会使用已存在的 WSL GOAI 环境，不安装或修改系统依赖。输出目录必须是新目录，防止覆盖旧结果。

```powershell
wsl -d Ubuntu-24.04-GOAI -- bash /mnt/e/goai/goai_robotics_workspace/slam/route_navigation_v1/run_wsl.sh -m unittest discover -s tests -v

wsl -d Ubuntu-24.04-GOAI -- bash /mnt/e/goai/goai_robotics_workspace/slam/route_navigation_v1/run_wsl.sh scripts/replay.py --session B --initial 0.5 0.1 0 0 --output /mnt/e/goai/goai_robotics_workspace/slam/route_navigation_v1/results/B_manual_rerun

wsl -d Ubuntu-24.04-GOAI -- bash /mnt/e/goai/goai_robotics_workspace/slam/route_navigation_v1/run_wsl.sh scripts/replay.py --session B --scenario mixed_faults --initial 0.5 0.1 0 0 --output /mnt/e/goai/goai_robotics_workspace/slam/route_navigation_v1/results/B_manual_faults

wsl -d Ubuntu-24.04-GOAI -- bash /mnt/e/goai/goai_robotics_workspace/slam/route_navigation_v1/run_wsl.sh scripts/assess_results.py /mnt/e/goai/goai_robotics_workspace/slam/route_navigation_v1/results/B_manual_faults
```

最后一步才加载地图构建轨迹做事后诊断；不会回填运行时位姿。混合故障覆盖单雷达/相机掉线、全部输入中断、600 ms 延迟、重复、1 s 时钟跳变、95% 点丢失、IMU 断档、VIO 15 m 跳变、随机掉帧。

`summary.json` 包含完整时间比例，故障区间不从分母删除。`assessment.json` 和 `validation.png` 是事后诊断。处理耗时来自不按真实时间节奏的离线回放，不能当成 Orin 端到端实时能力。

## 放行实际自动导航之前

需要补齐并通过：当前完整回放失败段、盲初始化/绑架恢复测试、未参与建图的新一趟独立走行、相机/雷达与机身刚性外参复核、楼梯及走廊中间路径核验、路线进度持久化/人工重新关联、定位与速度命令时效看门狗、S10 控制接口与实机停车/人工接管测试。

建议先把固定地图定位单独验收到可靠，再接 S10 自带步态策略的速度接口。不能因为路线 JSON 正确或静态图看起来重合，就跳过这些验收。
