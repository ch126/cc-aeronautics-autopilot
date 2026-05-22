-- =============================================================
--  autopilot/config.lua
--  Create:Aeronautics (Simulated Project) + CC:Tweaked
--
--  Peripheral blocks (place on the ship, connect via Wired Modem):
--    altitude_sensor  -> reads Y height
--    velocity_sensor  -> reads scalar speed
--    gimbal_sensor    -> reads pitch + roll angles
--    nav_table        -> reads heading angle (relative to north)
--
--  Control is via redstone analog output (0-15) on computer sides:
--    SIDE_THRUST_F / B = forward / backward propellers
--    SIDE_THRUST_U / D = upward  / downward lift
--    SIDE_YAW_L  / R   = yaw left / right thrusters
-- =============================================================

local Config = {}

-- ── Sensor peripheral names (nil = auto-scan by type) ─────────
-- Set to exact name shown by `scan` command if auto-detect fails.
Config.SENSOR_ALTITUDE  = nil   -- peripheral type: "altitude_sensor"
Config.SENSOR_VELOCITY  = nil   -- peripheral type: "velocity_sensor"
Config.SENSOR_GIMBAL    = nil   -- peripheral type: "gimbal_sensor"
Config.SENSOR_NAV       = nil   -- peripheral type: "navigation_table"

-- ── Redstone output sides ─────────────────────────────────────
-- Set each to the computer side facing the corresponding modem/propeller.
-- Sides: "top" "bottom" "left" "right" "front" "back"
Config.SIDE_THRUST_F = "front"   -- forward thrust
Config.SIDE_THRUST_B = "back"    -- reverse thrust (or nil if only fwd)
Config.SIDE_THRUST_U = "top"     -- upward lift
Config.SIDE_THRUST_D = "bottom"  -- downward thrust (or nil)
Config.SIDE_YAW_L    = "left"    -- yaw left
Config.SIDE_YAW_R    = "right"   -- yaw right

-- Redstone analog power range for propellers (0 = off, 15 = max)
-- Most propeller/fan blocks respond to full 0-15 analog range.
Config.RS_MAX = 15

-- ── Flight parameters ─────────────────────────────────────────
Config.TICK_RATE         = 0.05    -- loop interval (s)
Config.ARRIVAL_RADIUS    = 3.0     -- waypoint arrival distance (blocks) [dead reckoning]
Config.MAX_SPEED         = 8.0     -- max desired horizontal speed (m/s)
Config.YAW_THRESHOLD     = 5.0     -- heading error below this -> allow thrust (deg)
Config.ALT_THRESHOLD     = 1.0     -- altitude error below this -> stop lift (blocks)

-- ── PID gains (altitude) ─────────────────────────────────────
Config.PID_ALT = {
    kp = 1.5,
    ki = 0.05,
    kd = 0.8,
    integral_max = 5.0,
    output_max   = 15.0,
}

-- ── PID gains (horizontal speed) ─────────────────────────────
Config.PID_SPD = {
    kp = 2.0,
    ki = 0.1,
    kd = 0.5,
    integral_max = 5.0,
    output_max   = 15.0,
}

-- ── PID gains (heading / yaw) ─────────────────────────────────
Config.PID_HDG = {
    kp = 0.3,
    ki = 0.005,
    kd = 0.1,
    integral_max = 10.0,
    output_max   = 15.0,
}

-- ── Dead reckoning & sensor filtering ────────────────────────
-- EMA smoothing factor for speed before DR integration
--   Range 0.0-1.0: larger = faster response but more noise
--   0.25 = mild smoothing (~4-tick lag), 0.10 = heavy smoothing
Config.DR_EMA_SPEED  = 0.25

-- EMA smoothing factor for heading before DR integration
--   0.30 = mild smoothing; heading changes take ~3 ticks to settle
Config.DR_EMA_HDG    = 0.30

-- Minimum filtered speed (m/s) to allow DR integration
--   Below this, position is frozen (prevents drift at rest)
Config.DR_MIN_SPEED  = 0.15

return Config
