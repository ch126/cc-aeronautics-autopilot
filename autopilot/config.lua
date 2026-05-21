-- =============================================================
--  autopilot/config.lua
--  全局配置：外设名称、PID 参数、阈值
--  适用：CC:Tweaked + Create:Aeronautics  NeoForge 1.21.1
-- =============================================================

local Config = {}

-- ── 外设名称（按实际连接名称修改）──────────────────────────────
-- Create:Aeronautics 飞艇控制器（Helm / Airship Controller）
Config.HELM_NAME          = "create_aeronautics:helm_0"
-- 雷达/传感器（若安装了 Advanced Peripherals 或类似 mod）
Config.RADAR_NAME         = "peripheral_radar_0"
-- 是否有雷达外设（无则退化为盲飞模式）
Config.HAS_RADAR          = false

-- ── 飞行参数 ───────────────────────────────────────────────────
Config.TICK_RATE          = 0.05   -- 主循环间隔 (秒)  ≈ 20 tick/s
Config.ARRIVAL_RADIUS     = 2.0    -- 距目标 ≤ 此距离视为"到达" (格)
Config.MAX_SPEED          = 8.0    -- 最大速度上限 (格/s)
Config.MAX_VERTICAL_SPEED = 4.0    -- 最大垂直速度 (格/s)
Config.MAX_THROTTLE       = 1.0    -- 油门范围 [-1, 1]
Config.YAW_SPEED          = 60.0   -- 最大偏航角速度 (度/s)
Config.HEADING_THRESHOLD  = 5.0    -- 偏航误差 ≤ 此值才给前进推力 (度)

-- ── PID 参数（水平 X/Z）──────────────────────────────────────
Config.PID_H = {
    kp = 0.18,   -- 比例
    ki = 0.004,  -- 积分
    kd = 0.22,   -- 微分
    integral_max = 5.0,  -- 积分限幅（防饱和）
    output_max   = 1.0,
}

-- ── PID 参数（垂直 Y）────────────────────────────────────────
Config.PID_V = {
    kp = 0.30,
    ki = 0.005,
    kd = 0.25,
    integral_max = 3.0,
    output_max   = 1.0,
}

-- ── PID 参数（偏航 Yaw）──────────────────────────────────────
Config.PID_YAW = {
    kp = 0.8,
    ki = 0.01,
    kd = 0.15,
    integral_max = 30.0,
    output_max   = 1.0,
}

-- ── 避障参数 ─────────────────────────────────────────────────
Config.OBSTACLE = {
    detect_range    = 16,   -- 雷达扫描距离 (格)
    repulse_range   = 6,    -- 斥力生效距离 (格)
    repulse_gain    = 3.5,  -- 斥力强度系数
    attract_gain    = 1.0,  -- 引力归一化系数
    min_alt         = 5,    -- 最低安全飞行高度（地面以上）
    alt_step        = 4,    -- 抬升步进 (格)
    scan_dirs = {           -- 扫描方向（前/后/左/右/上/下）
        {1,0,0},{-1,0,0},{0,0,1},{0,0,-1},{0,1,0},{0,-1,0},
        -- 对角线
        {1,0,1},{-1,0,1},{1,0,-1},{-1,0,-1},
        {1,1,0},{-1,1,0},{0,1,1},{0,1,-1},
    },
}

-- ── 日志级别 ─────────────────────────────────────────────────
-- "DEBUG" | "INFO" | "WARN" | "ERROR"
Config.LOG_LEVEL = "INFO"

return Config
