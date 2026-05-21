-- =============================================================
--  autopilot/sensors.lua
--  Wraps all Create:Aeronautics CC peripherals into one object.
--
--  Peripheral types exposed by Simulated Project:
--    "altitude_sensor"    -> getHeight() : float  (world Y)
--    "velocity_sensor"    -> getVelocity() : float  (scalar m/s)
--    "gimbal_sensor"      -> getAngles() : {pitch_deg, roll_deg}
--    "navigation_table"   -> getRelativeAngle() : float (heading deg)
-- =============================================================

local Config = require("autopilot.config")

local Sensors = {}
Sensors.__index = Sensors

-- Find a peripheral by config name override or by type scan
local function findPeripheral(configName, typeName)
    if configName then
        local p = peripheral.wrap(configName)
        if p then return p end
    end
    return peripheral.find(typeName)
end

function Sensors.new()
    local self = setmetatable({}, Sensors)

    self.alt_p  = findPeripheral(Config.SENSOR_ALTITUDE, "altitude_sensor")
    self.vel_p  = findPeripheral(Config.SENSOR_VELOCITY, "velocity_sensor")
    self.gim_p  = findPeripheral(Config.SENSOR_GIMBAL,   "gimbal_sensor")
    self.nav_p  = findPeripheral(Config.SENSOR_NAV,      "navigation_table")

    -- Current readings (updated by :read())
    self.altitude = 0.0    -- world Y height (blocks)
    self.speed    = 0.0    -- scalar horizontal speed (m/s)
    self.pitch    = 0.0    -- nose-up positive (degrees)
    self.roll     = 0.0    -- right-roll positive (degrees)
    self.heading  = 0.0    -- compass heading (degrees, 0=north)

    -- Dead-reckoning estimated position (relative to start)
    self.dr_x     = 0.0
    self.dr_z     = 0.0

    return self
end

-- Returns a human-readable status of which sensors are connected
function Sensors:status()
    local function yn(p) return p and "OK" or "--" end
    return string.format(
        "alt:%s  vel:%s  gim:%s  nav:%s",
        yn(self.alt_p), yn(self.vel_p), yn(self.gim_p), yn(self.nav_p))
end

-- Read all sensors; dt is time since last call (for dead reckoning)
function Sensors:read(dt)
    if self.alt_p then
        local ok, v = pcall(self.alt_p.getHeight)
        if ok and v then self.altitude = v end
    end

    if self.vel_p then
        local ok, v = pcall(self.vel_p.getVelocity)
        if ok and v then self.speed = math.abs(v) end
    end

    if self.gim_p then
        local ok, v = pcall(self.gim_p.getAngles)
        if ok and v and type(v) == "table" then
            self.pitch = v[1] or 0.0
            self.roll  = v[2] or 0.0
        end
    end

    if self.nav_p then
        local ok, v = pcall(self.nav_p.getRelativeAngle)
        if ok and v then self.heading = v % 360 end
    end

    -- Dead reckoning: integrate speed along heading
    if dt and dt > 0 then
        local hr = math.rad(self.heading)
        -- Minecraft: +Z = south, +X = east; heading 0=north=-Z
        self.dr_x = self.dr_x + self.speed * math.sin(hr) * dt
        self.dr_z = self.dr_z - self.speed * math.cos(hr) * dt
    end
end

-- Reset dead-reckoning origin
function Sensors:resetDR()
    self.dr_x = 0.0
    self.dr_z = 0.0
end

return Sensors
