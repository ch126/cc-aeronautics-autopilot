-- =============================================================
--  car/config.lua
--  自动驾驶汽车配置
--
--  传感器外设（nil = 自动扫描）：
--    navigation_table  -> getRelativeAngle()  当前朝向 0-360
--    velocity_sensor   -> getVelocity()       当前速度 m/s
--    gps (可选)        -> getPosition()       返回 {x, y, z} 世界坐标
--
--  控制方式（红石模拟信号 0-15）：
--    SIDE_THROTTLE_F   前进油门
--    SIDE_THROTTLE_B   倒车/刹车
--    SIDE_STEER_L      左转
--    SIDE_STEER_R      右转
-- =============================================================

local Config = {}

-- ── 传感器外设名称 ────────────────────────────────────────────
Config.SENSOR_NAV       = nil   -- navigation_table
Config.SENSOR_VELOCITY  = nil   -- velocity_sensor
Config.SENSOR_GPS       = nil   -- GPS 外设（可选，有则用坐标，无则用 DR）

-- ── 红石输出面 ────────────────────────────────────────────────
Config.SIDE_THROTTLE_F  = "front"   -- 前进油门
Config.SIDE_THROTTLE_B  = "back"    -- 刹车/倒车
Config.SIDE_STEER_L     = "left"    -- 左转
Config.SIDE_STEER_R     = "right"   -- 右转

Config.RS_MAX           = 15

-- ── 行驶参数 ──────────────────────────────────────────────────
Config.TICK_RATE        = 0.05   -- 控制循环间隔 (s)
Config.MAX_SPEED        = 6.0    -- 最大巡航速度 (m/s)
Config.ARRIVAL_RADIUS   = 3.0    -- 到达判定半径 (blocks)
Config.STEER_THRESHOLD  = 3.0    -- 航向误差小于此值停止转向 (deg)

-- 接近目标时减速的距离
Config.BRAKE_ZONE       = 15.0   -- 在此距离内开始减速 (blocks)
Config.MIN_SPEED        = 1.0    -- 最低巡航速度 (m/s)

-- ── 死区推算参数 ──────────────────────────────────────────────
Config.DR_EMA_SPEED     = 0.3
Config.DR_EMA_HDG       = 0.3
Config.DR_MIN_SPEED     = 0.1

-- ── PID: 速度控制 ─────────────────────────────────────────────
Config.PID_SPD = {
    kp = 2.5,
    ki = 0.1,
    kd = 0.3,
    integral_max = 5.0,
    output_max   = 15.0,
}

-- ── PID: 转向控制 ─────────────────────────────────────────────
Config.PID_STEER = {
    kp = 0.25,
    ki = 0.002,
    kd = 0.08,
    integral_max = 8.0,
    output_max   = 15.0,
}

return Config
