# 地图临时路点与 Lightning 接入

导航服务 v1.32-reconnect；G12 App v1.21-map-waypoints（versionCode 21）。本包合并上一版尚未部署的 Lightning 接入，可从现有 v1.29、v1.30 或 v1.31 升级。本版仅修正 Orin 导航逻辑。G12 App v1.21 已安装，无需重装；Orin 更新已于 2026-09-22 部署并启动；完整测试和在线链路检查通过，实际行走待现场验证。详见 `部署记录.md`。

启动脚本已修正 tmux 名称前缀冲突：精确区分导航主服务与 Lightning 服务，重复启动时分别复用各自进程。

## 本次修正

1. 重定位后的航向参考：临时路点与原路线共用重新接入后的控制状态初始化，在地图对齐改变后丢弃旧航向参考，从新姿态建立参考。模拟中错误转向由约 -1.12 rad/s 变为正常的小幅修正 +0.008 rad/s，队列和导航运行状态保留。
2. 清空/删除最后一个临时点：与正常恢复导航共用前向路线接入，重新选择当前位置前方的剩余点，被绕过的点记为 skipped，原路线与 reached 保留。模拟中目标由身后的 P1 改为 P2。人工明确设定的当前目标仍具有优先级；暂停、人工接管和暂时无位姿时也覆盖了相应测试。

按用户要求，楼层与到点判定保持原实现。未改变速度范围、策略切换暂停或 PID 参数。APK 与上一版完全相同。

修复前后的模拟数据见 `verification/before-fixes.json` 和 `verification/after-fixes.json`；相对 v1.31 的改动见 `predeploy-fixes.diff`。

## 使用临时路点

1. 在导航地图右侧“临时后续点”中开启 **点选追加**，按希望经过的顺序点击地图。导航运行中立即更新后续轨迹；人工接管或暂停时只编辑路线，继续导航后才执行。
2. 黄色临时点与虚线表示临时路径。经过这些点后，接回原路线前方的目标点；原路线、已有路点及校准数据保持完整。
3. **点选替换**：下一次地图点击替换全部尚未经过的临时点。该模式保持开启，直到再次点击按钮退出或切换到其他点选模式。
4. **后续点 N**：查看未经过的点，选择某点后可在地图改点、修改坐标/高度或删除。**清空**取消剩余临时点，从当前位置重新接入前方剩余路线；人工指定的目标保持有效。
5. 再次点击“点选追加/替换”退出点选。拖动地图或双指缩放不会增加路点；临时点选与重定位点选互斥。

临时点的高度默认取当前高度筛选下最近的参考机身轨迹；多层区域可先选择低层/高层，再点选，并在队列中修正高度。坐标均为当前地图坐标。

人工接管期间，实际已经经过的临时点会从待执行队列移除；重定位跳变不会被当作人工运动轨迹。恢复导航时从当前位置连接剩余临时点。临时队列为空时沿用原来的前向接入逻辑。

临时点沿用原 Hermite 轨迹和 PID 跟踪，不为每个中间点增加停顿，不增加启用按钮、运动确认或速度限制。原有步态覆盖/预设逻辑继续有效；临时段继承当前原路线目标对应的策略，亦可手动覆盖。模式切换时仍保留此前要求的原地短暂停顿。

原路线完成后仍可添加临时点，使用原来的导航启动入口执行。重新选择路线或手动设置原路线进度会清空临时队列；临时点不写入正式路线，也不作为录制新路线保存。任意断电后不保证恢复临时队列；本包的正常升级流程会保存并恢复暂停时的队列。

## 本机验证

- 263 项测试全部通过，无跳过、无失败、无错误。包括临时点增删改、请求去重、原路线保留、连续经过临时点、长绕行后前向接入、人工接管后恢复、重定位跳变、暂停升级恢复队列，以及已有定位/控制/路线/策略测试。
- 二维运动响应模拟中，实际运行导航与 PID，依次经过两处临时点并回到原路线终点，临时点途中没有新增确认停顿。这是软件仿真，不是机器人实测。
- Android Java、资源、DEX 编译通过。APK 已用原 development.keystore 签名；证书 SHA-256 与上一版一致，可覆盖安装并保留应用数据。
- 上一版 Lightning 的实测数据适配验证记录保留在 `verification/lightning-v1.30/`，本次没有修改 Lightning C++ 算法内核；其接入细节见 `Lightning-v1.30-接入说明.md`。旧说明的测试数字对应上一版，本版以本节和 `verification/unit-results.json` 为准。

已完成 Orin ARM64 编译、ROS 实时数据链路及 G12 网络访问验证。G12 实际触控体验和真实机器人闭环导航仍待现场测试；Lightning 接入本身不保证消除里程计漂移。

## 后续重新部署

本包为差量包，目标为已有 `/home/wym/s10_navigation_app_v1`。不包含原签名私钥，不改写路线、预设、校准或运控网关文件。

在 Orin 将包解压到 `/home/wym/goai-navigation-deployed-20260922`，先编译并测试：

```bash
cd /home/wym/goai-navigation-deployed-20260922
bash build_lightning.sh
python3 deploy_navigation.py --check-only
```

Lightning 编译到独立的 `/home/wym/s10_lightning_runtime_v1`。部署测试在副本中执行，使用原 ROS 环境与隔离 domain；不会启动机器人运动。

在 App 暂停当前运动后更新服务：

```bash
cd /home/wym/goai-navigation-deployed-20260922
python3 deploy_navigation.py
bash /home/wym/s10_navigation_app_v1/start_all.sh
```

也可用 `bash install.sh` 合并编译与部署。脚本沿用原有部署暂停检查，备份被替换文件、当前路线和进度；重启后保持暂停。源文件有未知改动时报告具体文件，不覆盖用户编辑。首次启用 Lightning 后在 App 进行重定位，再按需开始导航或人工控制。

当前 G12 已安装同一份 APK，无需重装。包内保留 `GOAI-Navigation.apk` 供其他设备安装：

```powershell
adb install -r GOAI-Navigation.apk
```

后续一键启动 Orin 服务仍为：

```bash
bash /home/wym/s10_navigation_app_v1/start_all.sh
```

## 回滚

部署输出包含 `logs/pre-map-waypoints-xxxx` 备份目录。暂停运动后执行：

```bash
python3 deploy_navigation.py --rollback /home/wym/s10_navigation_app_v1/logs/pre-map-waypoints-xxxx
```

回滚保留当时的原路线进度，不覆盖后续校准和预设修改。旧服务不支持临时队列，回退到 v1.29/v1.30 时临时点不再执行。G12 App 若也需要降级，使用原版 APK；Android 可能不允许直接降级安装，本包不执行卸载或清除应用数据。

源码差量在 `app/`，逐文件兼容基线及新文件哈希在 `navigation-manifest.json`，差异在 `changes.diff`；整个包的校验清单为 `PACKAGE_SHA256.json`。
