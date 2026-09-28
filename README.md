# GOAI Navigation 1.33

GOAI 四足机器人导航系统的源码与实现交付包，包含 Orin 导航服务、G12 Android 控制端、运控网关、Lightning LIO 运行时源码、地图与路线配置、部署增量，以及对应的测试和验证材料。

> [!IMPORTANT]
> 本仓库是基于既有设备环境的工程交付，不是空白 Orin 的整机镜像，也不是适用于所有历史版本的通用安装器。部署前请阅读[部署与复现](docs/部署与复现.md)，并先备份现场配置和运行进度。

## 版本概览

| 组件 | 版本 / 状态 |
|---|---|
| Orin 导航服务 | `1.33-auto-map` |
| G12 Android App | `1.22-offline-route` |
| 运控网关 | `1.18-step-move` |
| 路线校准 | revision `103` |
| 预设 | revision `7` |
| 最近机器人在线验证 | 2026-09-22 |
| 交付包整理日期 | 2026-09-26 |

实现基线为 Ubuntu 24.04 ARM64、ROS 2 Jazzy 和已有 S10 运行环境。2026-09-26 的整理工作未重新连接或修改机器人。

## 主要内容

- Orin 端导航、定位、规划、轨迹跟踪、PID 控制及任务仲裁源码
- Lightning LIO 运行时源码、ROS 2 消息包和离线构建脚本
- G12 Android 导航 App 源码与已签名 APK
- 自动地图修正、人工附近重定位和连续位姿融合实现
- 路线、校准、预设及逐点/逐段配置快照
- 自动地图、定位、LIO 对比、重连与 APK 验证证据
- 技术报告、部署手册和 SHA-256 完整性清单

## 仓库结构

| 路径 | 内容 |
|---|---|
| [`report/`](report/) | 技术报告及引用图表，提供 Markdown 和 PDF 版本 |
| [`source/slam/`](source/slam/) | 导航 App、网关、Android 源码、依赖工程及地图资产 |
| [`source/lightning/`](source/lightning/) | 已用 Lightning 运行时源码、构建脚本和源码清单 |
| [`release/android/`](release/android/) | 已签名的 G12 Android APK |
| [`release/orin-auto-map-1.33/`](release/orin-auto-map-1.33/) | 从既有 1.32 环境升级到 1.33 的增量与部署脚本 |
| [`config-snapshot/`](config-snapshot/) | 校准 103、预设 7、正式路线及 CSV 快照 |
| [`evidence/`](evidence/) | 软件测试、历史部署、定位分析、LIO 对比和 APK 验证记录 |
| [`docs/`](docs/) | 模块索引及部署复现说明 |
| [`tools/`](tools/) | 交付包完整性校验工具 |

详细源码职责见[模块索引](docs/模块索引.md)。

## 快速检查

克隆仓库后先验证交付文件是否完整：

```bash
python3 tools/verify_delivery.py
```

预期输出：

```json
{
  "ok": true,
  "checked": 639,
  "failures": []
}
```

该命令只在本地读取文件并核对大小与 SHA-256，不连接机器人，也不会修改现场环境。

建议按以下顺序阅读：

1. [GOAI 导航技术报告](report/GOAI导航技术报告.md)
2. [部署与复现](docs/部署与复现.md)
3. [模块索引](docs/模块索引.md)

## 已有设备启动

已经部署 1.33 的 Orin 可使用现有入口启动：

```bash
bash /home/wym/s10_navigation_app_v1/start_all.sh
```

只读状态检查：

```bash
python3 /home/wym/s10_navigation_app_v1/startup_status.py
python3 /home/wym/s10_navigation_app_v1/lightning/service.py status
```

启动脚本会恢复 G12 回程路由并启动或复用运控网关、Lightning 和导航主服务，但不会主动发起导航或控制机器人起立。完整网络拓扑、依赖和升级步骤见[部署与复现](docs/部署与复现.md)。

## 从 1.32 升级到 1.33

先在独立目录校验交付包，再执行隔离测试：

```bash
python3 tools/verify_delivery.py
cd release/orin-auto-map-1.33
python3 deploy_auto_map.py
```

不带参数时，部署脚本仅在副本中执行导航测试，使用 ROS domain 92，不修改在线服务。确认 App 已暂停、当前控制已结束且网关输出为零后，才可执行实际部署：

```bash
python3 deploy_auto_map.py --deploy
```

> [!WARNING]
> 不要直接覆盖整个源码目录，也不要用本仓库中的历史进度或配置覆盖设备上的较新状态。部署脚本面向已有 1.32 环境，并带有源码哈希和运行状态检查。

## Android APK

```powershell
adb devices
adb install -r release/android/GOAI-Navigation-1.22-offline-route.apk
```

`-r` 会保留 App 数据。已签名 APK 使用原证书；仓库不包含 APK 签名私钥。新设备仍需完成既有的独立配对流程。

## 验证状态与已知边界

- 自动地图修正已通过 373 项 Orin 导航测试及真实静态点云验证。
- APK 缓存另有 12 项测试通过；该测试集合与导航测试相互独立，不应合并计数。
- 当前交付没有新版行走验收记录。
- 常态约 0.13–0.19 秒观测延迟的控制时刻外推尚未实现。
- `config-snapshot` 的正式路线为校准 103；APK 离线初始路线为旧的校准 64，两者不可视为同一版本。
- 原始大体积日志未全部纳入仓库；[`evidence/raw-archive-index.json`](evidence/raw-archive-index.json) 仅提供索引，无法据此直接重跑完整历史点云试验。

详细结论和证据链以[技术报告](report/GOAI导航技术报告.md)及 [`evidence/`](evidence/) 中标注日期的记录为准。

## 安全与敏感信息

本交付包不包含设备密钥、APK 签名私钥、sudo 密码或私有配对配置。部署脚本默认保留 Orin 私有目录，不会自动导入配置快照或恢复历史任务进度。

机器人运动和在线升级具有现实安全风险。执行部署或导航前，应确保机器人处于可控环境、具备急停条件，并由熟悉现场系统的人员确认暂停状态、网络配置和控制输出。

## 许可证

仓库根目录目前未声明统一的开源许可证，因此请勿默认将整个项目视为可自由复制、修改或再分发的软件。第三方组件的许可证以其各自目录中的许可证文件为准，例如 Lightning 运行时的 [`LICENSE.txt`](source/lightning/runtime-source/vendor/lightning/LICENSE.txt)。
