-- =============================================================
--  autopilot/config.lua
--  Global config: peripheral names, PID gains, thresholds
--  Target: CC:Tweaked + Create:Aeronautics  NeoForge 1.21.1
-- =============================================================

local Config = {}

-- -- Peripheral ------------------------------------------------
-- Create:Aeronautics Airship Helm peripheral type
-- Run `peripheral.getNames()` in-game to find the exact name.
-- Common values:
--   "create_aeronautics:airship_helm"
--   "create_aeronautics:tilt_airship_helm"
--   "airshipHelm"
-- Set to nil to auto-scan all attached peripherals.
Config.HELM_TYPE = nil  -- nil = auto-detect

-- If auto-detect finds multiple, it picks the first.
-- Set to exact peripheral side/name to force a specific one, e.g. "right"
Config.HELM_SIDE = nil  -- nil = auto-detect

-- Radar peripheral (Advanced Peripherals). Set HAS_RADAR=false if not installed.
Config.RADAR_NAME = "neuralInterface"
Config.HAS_RADAR  = false

-- -- Flight parameters -----------------------------------------
Config.TICK_RATE          = 0.05   -- main loop interval (s), ~20 tps
Config.ARRIVAL_RADIUS     = 2.0    -- arrival threshold (blocks)
Config.MAX_SPEED          = 8.0    -- max speed cap (blocks/s)
Config.MAX_VERTICAL_SPEED = 4.0    -- max vertical speed (blocks/s)
Config.MAX_THROTTLE       = 1.0    -- throttle range [-1, 1]
Config.YAW_SPEED          = 60.0   -- max yaw rate (deg/s)
Config.HEADING_THRESHOLD  = 5.0    -- yaw error below this allows forward thrust (deg)

-- -- PID gains (horizontal X/Z) --------------------------------
Config.PID_H = {
    kp = 0.18,
    ki = 0.004,
    kd = 0.22,
    integral_max = 5.0,
    output_max   = 1.0,
}

-- -- PID gains (vertical Y) ------------------------------------
Config.PID_V = {
    kp = 0.30,
    ki = 0.005,
    kd = 0.25,
    integral_max = 3.0,
    output_max   = 1.0,
}

-- -- PID gains (yaw) -------------------------------------------
Config.PID_YAW = {
    kp = 0.8,
    ki = 0.01,
    kd = 0.15,
    integral_max = 30.0,
    output_max   = 1.0,
}

-- -- Obstacle avoidance ----------------------------------------
Config.OBSTACLE = {
    detect_range  = 16,
    repulse_range = 6,
    repulse_gain  = 3.5,
    attract_gain  = 1.0,
    min_alt       = 5,
    alt_step      = 4,
    scan_dirs = {
        {1,0,0},{-1,0,0},{0,0,1},{0,0,-1},{0,1,0},{0,-1,0},
        {1,0,1},{-1,0,1},{1,0,-1},{-1,0,-1},
        {1,1,0},{-1,1,0},{0,1,1},{0,1,-1},
    },
}

-- -- Log level -------------------------------------------------
Config.LOG_LEVEL = "INFO"  -- DEBUG | INFO | WARN | ERROR

-- -- Velocity control mode (MODE A) ---------------------------
-- Used when helm exposes setVelocity(vx, vy, vz).
-- VEL_KP : proportional gain  position_error -> desired_speed
--          e.g. 0.5 means 10-block error -> 5 m/s target speed
-- MAX_SPEED is already defined above and acts as the speed cap.
Config.VEL_KP = 0.5  -- position error -> velocity gain

return Config
