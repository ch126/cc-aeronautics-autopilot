# ✈ Create:Aeronautics PID 自动驾驶系统

> **NeoForge 1.21.1 | CC:Tweaked | Create | Create:Aeronautics**

在 Minecraft 中为 Create:Aeronautics 飞艇实现 **PID 三轴控制 + 人工势场避障 + 图形化 TUI 驾驶界面**。

---

## 📦 依赖 Mod

| Mod | 版本要求 |
|-----|---------|
| [NeoForge](https://neoforged.net/) | 1.21.1 |
| [CC:Tweaked](https://github.com/cc-tweaked/CC-Tweaked) | ≥ 1.21.1-1.114 |
| [Create](https://github.com/Creators-of-Create/Create) | ≥ 6.0 (1.21.1) |
| [Create: Aeronautics](https://github.com/Creators-of-Create/Aeronautics) | Simulated Project |

---

## 🗂 项目结构

```
auto_drive/
├── startup.lua               ← 开机自启（放到电脑根目录）
├── install.lua               ← 一键网络部署（可选）
└── autopilot/
    ├── config.lua            ← PID 参数 & 外设名称配置
    ├── vec3.lua              ← 三维向量运算库
    ├── pid.lua               ← PID 控制器（单轴 + 三轴封装）
    ├── obstacle.lua          ← 人工势场法避障模块
    ├── nav.lua               ← 导航管理器（核心）
    ├── gui.lua               ← 图形化 TUI 界面
    └── main.lua              ← 主程序入口
```

---

## 🚀 快速开始

### 1. 部署文件

将整个 `autopilot/` 目录和 `startup.lua` 上传到 CC:Tweaked **Advanced Computer** 的根目录。

或在游戏内运行（需配置 `install.lua` 中的仓库 URL）：
```
lua install.lua
```

### 2. 配置外设

编辑 `autopilot/config.lua`，填写飞艇 Helm 的外设名称：
```lua
Config.HELM_NAME = "create_aeronautics:helm_0"  -- 按实际修改
```
> 在电脑终端输入 `peripheral list` 查询已连接外设名称。

### 3. 组装飞艇

1. 用 Create 积木搭建飞艇结构
2. 放置 **Aeronautics Helm（舵）**
3. 将 **Advanced Computer** 放置在飞艇上并连接到 Helm
4. 右键 Helm 组装飞艇

### 4. 运行

```lua
lua autopilot/main.lua
```
或将 `startup.lua` 放置于根目录开机自动启动。

---

## 🖥 图形界面

```
╔══════════════════════════════════════════════════════════╗
║  ✈  Create:Aeronautics PID 自动驾驶  v1.0               ║
╠══════════════════════╦═══════════════════════════════════╣
║  ┌─── 飞行状态 ─────┐║ ┌──── 航点队列 ─────────────────┐║
║  │ 状 态  ► NAV     │║ │ ○[01]  100.0   80.0  -200.0  │║
║  │ 位 置  X=.. Y=.. │║ │►[02]  200.0   80.0     0.0  │║
║  │ 速 度  3.2 m/s   │║ │ ○[03]  -50.0  70.0   150.0  │║
║  └──────────────────┘║ └───────────────────────────────┘║
╠══════════════════════╩═══════════════════════════════════╣
║  ████████████░░░░░  航点 2/3  66.7%  剩余 87.3 格       ║
╠══════════════════════════════════════════════════════════╣
║  [ GO ] [ WP+ ] [ CLR ] [ LIST ] [ START ] [ STOP ] [?] ║
╠══════════════════════════════════════════════════════════╣
║ > goto 100 80 -200▌                                      ║
╠══════════════════════════════════════════════════════════╣
║ [00.3] ✓ 目标设定 (100,80,-200)                         ║
║ [00.5] ► 导航已启动                                      ║
╚══════════════════════════════════════════════════════════╝
```

### 按钮说明

| 按钮 | 功能 |
|------|------|
| **GO** | 弹出坐标输入框，直飞到指定坐标 |
| **WP+** | 弹出坐标输入框，追加一个航点 |
| **CLR** | 清空所有航点 |
| **LIST** | 在日志区显示航点数量 |
| **START** | 开始按队列顺序导航 |
| **STOP** | 立即停止，推力归零 |
| **?** | 显示帮助信息 |

---

## ⌨ 命令列表

| 命令 | 说明 |
|------|------|
| `goto <x> <y> <z>` | 清空航点并直飞到指定坐标 |
| `wp add <x> <y> <z>` | 追加一个航点到队列 |
| `wp clear` | 清空航点队列 |
| `start` | 开始导航 |
| `stop` | 停止飞行 |
| `tune h <kp> <ki> <kd>` | 在线调整水平 PID 参数 |
| `tune v <kp> <ki> <kd>` | 在线调整垂直 PID 参数 |
| `help` | 显示帮助 |
| `exit` | 退出程序 |

---

## ⚙ 系统架构

### PID 控制器

三轴独立 PID，X/Z 共用水平参数，Y 单独垂直参数：

$$u(t) = K_p e(t) + K_i \int e \, dt + K_d \frac{de}{dt}$$

### 避障（人工势场法）

$$\vec{F}_{total} = \vec{F}_{attract} + \vec{F}_{repulse}$$

$$F_{repulse} = k_r \left(\frac{1}{d} - \frac{1}{d_0}\right) \frac{1}{d^2}$$

- 有雷达外设：全方向扫描 16 格，精细避障
- 无雷达（盲飞）：自动维持最低安全高度

### 调参建议

| 现象 | 调整 |
|------|------|
| 飞行摇摆/震荡 | 降低 `kp`，增大 `kd` |
| 到达精度差 | 增大 `kp`，加入小 `ki` |
| 转向过冲 | 降低 `PID_YAW.kp` |
| 垂直爬升慢 | 增大 `PID_V.kp` |
| 避障太激进 | 降低 `repulse_gain` |

---

## 📄 License

MIT License
