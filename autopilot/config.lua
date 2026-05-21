-- =============================================================
--  autopilot/config.lua
  -- PID
  -- CC:Tweaked + Create:Aeronautics  NeoForge 1.21.1
-- =============================================================

local Config = {}


  -- Create:Aeronautics Helm / Airship Controller
Config.HELM_NAME          = "create_aeronautics:helm_0"
  -- / Advanced Peripherals  mod
Config.RADAR_NAME         = "peripheral_radar_0"

Config.HAS_RADAR          = false


Config.TICK_RATE          = 0.05  -- ()   20 tick/s
Config.ARRIVAL_RADIUS     = 2.0  -- "" ()
Config.MAX_SPEED          = 8.0  -- (/s)
Config.MAX_VERTICAL_SPEED = 4.0  -- (/s)
Config.MAX_THROTTLE       = 1.0  -- [-1, 1]
Config.YAW_SPEED          = 60.0  -- (/s)
Config.HEADING_THRESHOLD  = 5.0  -- ()

  -- PID  X/Z
Config.PID_H = {
    kp = 0.18,
    ki = 0.004,
    kd = 0.22,
    integral_max = 5.0,
    output_max   = 1.0,
}

  -- PID  Y
Config.PID_V = {
    kp = 0.30,
    ki = 0.005,
    kd = 0.25,
    integral_max = 3.0,
    output_max   = 1.0,
}

  -- PID  Yaw
Config.PID_YAW = {
    kp = 0.8,
    ki = 0.01,
    kd = 0.15,
    integral_max = 30.0,
    output_max   = 1.0,
}


Config.OBSTACLE = {
    detect_range    = 16,  -- ()
    repulse_range   = 6,  -- ()
    repulse_gain    = 3.5,
    attract_gain    = 1.0,
    min_alt         = 5,
    alt_step        = 4,  -- ()
    scan_dirs = {  -- /////
        {1,0,0},{-1,0,0},{0,0,1},{0,0,-1},{0,1,0},{0,-1,0},

        {1,0,1},{-1,0,1},{1,0,-1},{-1,0,-1},
        {1,1,0},{-1,1,0},{0,1,1},{0,1,-1},
    },
}


-- "DEBUG" | "INFO" | "WARN" | "ERROR"
Config.LOG_LEVEL = "INFO"

return Config
