-- =============================================================
--  autopilot/sensors.lua
--  Wraps all Create:Aeronautics CC peripherals into one object.
--
--  Peripheral types exposed by Simulated Project:
--    "altitude_sensor"    -> getHeight()        : float  (world Y)
--    "velocity_sensor"    -> getVelocity()       : float  (scalar m/s, signed)
--    "gimbal_sensor"      -> getAngles()         : {pitch_deg, roll_deg}
--    "navigation_table"   -> getRelativeAngle()  : float  (heading 0-360)
--
--  Dead-reckoning model:
--    horiz_speed = |speed| * cos(pitch_rad)   -- strip vertical component
--    vert_speed  = |speed| * sin(pitch_rad)   -- for reference only (Y from sensor)
--    dr_x += horiz_speed * sin(hdg_rad) * dt  -- east/west  (+X = east)
--    dr_z -= horiz_speed * cos(hdg_rad) * dt  -- north/south(-Z = north)
--    altitude = direct from altitude_sensor    -- no vertical accumulation error
--
--  Filtering:
--    EMA (exponential moving average) on speed & heading before integration
--    Dead-zone: skip DR update if filtered speed < DR_MIN_SPEED
-- =============================================================

local Config = require("autopilot.config")

-- ── Tunable filtering constants ───────────────────────────────
-- EMA smoothing factor: 1 = no filter, 0.05 = very heavy
local EMA_SPEED   = Config.DR_EMA_SPEED   or 0.25
local EMA_HDG     = Config.DR_EMA_HDG     or 0.30
-- Below this speed (m/s), skip DR integration (stop drift)
local DR_MIN_SPEED = Config.DR_MIN_SPEED  or 0.15

-- ── Helpers ───────────────────────────────────────────────────
local function findPeripheral(configName, typeName)
    if configName then
        local p = peripheral.wrap(configName)
        if p then return p end
    end
    return peripheral.find(typeName)
end

-- Shortest angular delta for heading EMA (-180..180)
local function hdgDelta(target, current)
    local d = (target - current) % 360
    if d > 180 then d = d - 360 end
    return d
end

-- ── Sensors object ────────────────────────────────────────────
local Sensors = {}
Sensors.__index = Sensors

function Sensors.new()
    local self = setmetatable({}, Sensors)

    -- Peripherals
    self.alt_p = findPeripheral(Config.SENSOR_ALTITUDE, "altitude_sensor")
    self.vel_p = findPeripheral(Config.SENSOR_VELOCITY, "velocity_sensor")
    self.gim_p = findPeripheral(Config.SENSOR_GIMBAL,   "gimbal_sensor")
    self.nav_p = findPeripheral(Config.SENSOR_NAV,      "navigation_table")

    -- ── Live sensor readings (raw, updated every tick) ────────
    self.altitude     = 0.0   -- world Y (blocks) — absolute, from sensor
    self.speed        = 0.0   -- total scalar speed (m/s)
    self.pitch        = 0.0   -- nose-up positive (deg)
    self.roll         = 0.0   -- right-roll positive (deg)
    self.heading      = 0.0   -- compass heading (deg, 0=north CW)

    -- ── EMA-filtered values used for integration ──────────────
    self._filt_speed  = 0.0   -- filtered |speed|
    self._filt_hdg    = 0.0   -- filtered heading (wrapped)

    -- ── Derived velocity components (informational) ───────────
    self.horiz_speed  = 0.0   -- horizontal ground speed (m/s)
    self.vert_speed   = 0.0   -- vertical speed estimate (m/s, + = climbing)
    self.vx           = 0.0   -- east  velocity component
    self.vz           = 0.0   -- south velocity component (+ = south)

    -- ── Dead-reckoning position (blocks, relative to origin) ──
    self.dr_x         = 0.0   -- east  offset (+X)
    self.dr_z         = 0.0   -- south offset (+Z)
    -- altitude is absolute from sensor, no DR needed for Y

    -- ── Statistics ────────────────────────────────────────────
    self.dr_dist      = 0.0   -- total horizontal distance travelled

    -- ── Bootstrap flag ───────────────────────────────────────
    self._initialized = false

    return self
end

-- ── Sensor connection status string ──────────────────────────
function Sensors:status()
    local function yn(p) return p and "OK" or "--" end
    return string.format(
        "alt:%s  vel:%s  gim:%s  nav:%s",
        yn(self.alt_p), yn(self.vel_p), yn(self.gim_p), yn(self.nav_p))
end

-- ── Main read + integration tick ──────────────────────────────
-- Call once per control tick; dt = seconds since last call
function Sensors:read(dt)
    -- 1. Raw sensor reads
    if self.alt_p then
        local ok, v = pcall(self.alt_p.getHeight)
        if ok and type(v) == "number" then self.altitude = v end
    end

    local raw_speed = self.speed
    if self.vel_p then
        local ok, v = pcall(self.vel_p.getVelocity)
        if ok and type(v) == "number" then
            raw_speed = math.abs(v)
            self.speed = raw_speed
        end
    end

    if self.gim_p then
        local ok, v = pcall(self.gim_p.getAngles)
        if ok and type(v) == "table" then
            self.pitch = tonumber(v[1]) or 0.0
            self.roll  = tonumber(v[2]) or 0.0
        end
    end

    local raw_hdg = self.heading
    if self.nav_p then
        local ok, v = pcall(self.nav_p.getRelativeAngle)
        if ok and type(v) == "number" then
            raw_hdg = v % 360
            self.heading = raw_hdg
        end
    end

    -- 2. Bootstrap filtered values on first read
    if not self._initialized then
        self._filt_speed = raw_speed
        self._filt_hdg   = raw_hdg
        self._initialized = true
    end

    -- 3. EMA filtering
    --    Speed: simple EMA
    self._filt_speed = self._filt_speed + EMA_SPEED * (raw_speed - self._filt_speed)

    --    Heading: EMA on the shortest angular path (handles 359->1 wrap)
    local hdg_err = hdgDelta(raw_hdg, self._filt_hdg)
    self._filt_hdg = (self._filt_hdg + EMA_HDG * hdg_err) % 360

    -- 4. Decompose speed into horizontal / vertical components
    --    pitch > 0 = nose up => part of speed is vertical
    local pitch_rad = math.rad(self.pitch)
    self.horiz_speed = self._filt_speed * math.cos(pitch_rad)
    self.vert_speed  = self._filt_speed * math.sin(pitch_rad)

    -- 5. Dead-reckoning integration
    --    Only integrate when speed above dead-zone and dt is sane
    if dt and dt > 0 and dt < 1.0 and self.horiz_speed >= DR_MIN_SPEED then
        local hr  = math.rad(self._filt_hdg)
        local dxdt = self.horiz_speed * math.sin(hr)
        local dzdt = -self.horiz_speed * math.cos(hr)  -- -Z = north

        self.vx = dxdt
        self.vz = dzdt

        local step_x = dxdt * dt
        local step_z = dzdt * dt
        self.dr_x = self.dr_x + step_x
        self.dr_z = self.dr_z + step_z
        self.dr_dist = self.dr_dist + math.sqrt(step_x*step_x + step_z*step_z)
    else
        self.vx = 0.0
        self.vz = 0.0
    end
end

-- ── Reset dead-reckoning origin ───────────────────────────────
function Sensors:resetDR()
    self.dr_x    = 0.0
    self.dr_z    = 0.0
    self.dr_dist = 0.0
end

return Sensors
