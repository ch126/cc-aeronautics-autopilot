-- =============================================================
--  autopilot/nav.lua  (Create:Aeronautics Simulated Project)
--
--  Reads sensors, runs PID, outputs redstone analog signals.
--
--  Control axes:
--    ALTITUDE  : PID(target_alt, alt_sensor) -> rs analog on SIDE_THRUST_U/D
--    SPEED     : PID(target_spd, vel_sensor) -> rs analog on SIDE_THRUST_F/B
--    HEADING   : P(target_hdg, nav_table)    -> rs analog on SIDE_YAW_L/R
--
--  Flight modes:
--    IDLE     - all outputs zero
--    HOVER    - hold altitude, zero target speed
--    FLY      - hold altitude + heading + speed
--    GOTO     - dead-reckoning waypoint (heading + speed until dist reached)
--    MANUAL   - raw rs output controlled by caller (main.lua)
-- =============================================================

local PIDMod  = dofile("/autopilot/pid.lua")
local Sensors = dofile("/autopilot/sensors.lua")
local Config  = dofile("/autopilot/config.lua")

local Nav = {}
Nav.__index = Nav

Nav.MODE = {
    IDLE   = "IDLE",
    HOVER  = "HOVER",
    FLY    = "FLY",
    GOTO   = "GOTO",
    MANUAL = "MANUAL",
}

-- ── Redstone helper ───────────────────────────────────────────
-- power: 0.0-1.0 float -> mapped to 0-RS_MAX integer
local function rsSet(side, power)
    if not side then return end
    local v = math.floor(math.max(0, math.min(1, power)) * Config.RS_MAX + 0.5)
    rs.setAnalogOutput(side, v)
end

local function rsOff(side)
    if not side then return end
    rs.setAnalogOutput(side, 0)
end

local function normalizeAngle(a)
    a = a % 360
    if a > 180 then a = a - 360 end
    return a
end

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end

-- ── Constructor ───────────────────────────────────────────────
function Nav.new()
    local sens = Sensors.new()

    local pid_alt = PIDMod.PID.new(Config.PID_ALT)
    local pid_spd = PIDMod.PID.new(Config.PID_SPD)
    local pid_hdg = PIDMod.PID.new(Config.PID_HDG)

    local self = setmetatable({
        sensors   = sens,
        pid_alt   = pid_alt,
        pid_spd   = pid_spd,
        pid_hdg   = pid_hdg,

        mode      = Nav.MODE.IDLE,
        msg       = "Standby | " .. sens:status(),

        -- Setpoints
        target_alt = nil,    -- meters (world Y)
        target_spd = 0.0,    -- m/s
        target_hdg = nil,    -- degrees (0=north)

        -- GOTO waypoint (dead-reckoning, relative coords)
        wp_dx      = 0.0,
        wp_dz      = 0.0,
        wp_dist    = 0.0,

        elapsed    = 0.0,
    }, Nav)

    -- Do an initial sensor read to populate altitude/heading
    self.sensors:read(0)
    return self
end

-- ── Public setters ────────────────────────────────────────────
function Nav:setAltitude(y)
    self.target_alt = y
    self.pid_alt:reset()
    self.msg = string.format("Alt target: %.1f m", y)
end

function Nav:setSpeed(s)
    self.target_spd = math.max(0, s)
    self.pid_spd:reset()
end

function Nav:setHeading(h)
    self.target_hdg = h % 360
    self.pid_hdg:reset()
    self.msg = string.format("Heading target: %.1f deg", h)
end

function Nav:hover()
    self:setAltitude(self.sensors.altitude)
    self:setSpeed(0)
    self.target_hdg = self.sensors.heading
    self.pid_alt:reset(); self.pid_spd:reset(); self.pid_hdg:reset()
    self.mode = Nav.MODE.HOVER
    self.msg  = string.format("Hover at Y=%.1f", self.sensors.altitude)
end

function Nav:fly(alt, hdg, spd)
    self.target_alt = alt or self.sensors.altitude
    self.target_hdg = (hdg or self.sensors.heading) % 360
    self.target_spd = spd or Config.MAX_SPEED
    self.pid_alt:reset(); self.pid_spd:reset(); self.pid_hdg:reset()
    self.mode = Nav.MODE.FLY
    self.msg  = string.format("Fly alt=%.0f hdg=%.0f spd=%.1f",
        self.target_alt, self.target_hdg, self.target_spd)
end

-- Relative waypoint (dead-reckoning): dx=east, dz=south in blocks
function Nav:goto_rel(dx, dz, alt)
    self.sensors:resetDR()
    self.wp_dx   = dx
    self.wp_dz   = dz
    self.wp_dist = math.sqrt(dx*dx + dz*dz)
    self.target_alt = alt or self.sensors.altitude
    self.target_spd = math.min(Config.MAX_SPEED, self.wp_dist / 3)
    -- Heading towards target: atan2(dx, -dz) since north=-Z
    self.target_hdg = (math.deg(math.atan(dx, -dz))) % 360
    self.pid_alt:reset(); self.pid_spd:reset(); self.pid_hdg:reset()
    self.mode = Nav.MODE.GOTO
    self.msg  = string.format("Goto dX=%.0f dZ=%.0f alt=%.0f", dx, dz, self.target_alt)
end

function Nav:stop()
    self:_allStop()
    self.mode = Nav.MODE.IDLE
    self.msg  = "Stopped"
end

-- ── Update (called every tick) ────────────────────────────────
function Nav:update(dt)
    self.elapsed = self.elapsed + dt
    self.sensors:read(dt)

    if self.mode == Nav.MODE.IDLE or self.mode == Nav.MODE.MANUAL then
        return
    end

    -- Check GOTO arrival
    if self.mode == Nav.MODE.GOTO then
        local rem_x = self.wp_dx - self.sensors.dr_x
        local rem_z = self.wp_dz - self.sensors.dr_z
        local rem   = math.sqrt(rem_x*rem_x + rem_z*rem_z)
        if rem <= Config.ARRIVAL_RADIUS then
            self:hover()
            self.msg = string.format("Arrived! (DR err ~%.0f blk)", rem)
            return
        end
        -- Re-steer towards remaining vector
        self.target_hdg = (math.deg(math.atan(rem_x, -rem_z))) % 360
        -- Slow down near end
        self.target_spd = clamp(rem / 3, 0.5, Config.MAX_SPEED)
        self.msg = string.format("Goto rem=%.0f blk  hdg=%.0f", rem, self.target_hdg)
    end

    self:_controlStep(dt)
end

-- ── Control step ─────────────────────────────────────────────
function Nav:_controlStep(dt)
    local s = self.sensors

    -- ── Altitude ──────────────────────────────────────────────
    if self.target_alt then
        local alt_err = self.target_alt - s.altitude
        local lift    = self.pid_alt:compute(self.target_alt, s.altitude, dt)
        -- lift > 0 -> go up, lift < 0 -> go down
        if lift >= 0 then
            rsSet(Config.SIDE_THRUST_U, lift / Config.RS_MAX)
            rsOff(Config.SIDE_THRUST_D)
        else
            rsOff(Config.SIDE_THRUST_U)
            rsSet(Config.SIDE_THRUST_D, (-lift) / Config.RS_MAX)
        end
    else
        rsOff(Config.SIDE_THRUST_U)
        rsOff(Config.SIDE_THRUST_D)
    end

    -- ── Heading ───────────────────────────────────────────────
    local hdg_ok = true
    if self.target_hdg then
        local hdg_err = normalizeAngle(self.target_hdg - s.heading)
        local yaw_out = clamp(self.pid_hdg:compute(0, -hdg_err, dt), -1, 1)
        -- yaw_out > 0 -> turn right, < 0 -> turn left
        if yaw_out >= 0 then
            rsSet(Config.SIDE_YAW_R, yaw_out)
            rsOff(Config.SIDE_YAW_L)
        else
            rsOff(Config.SIDE_YAW_R)
            rsSet(Config.SIDE_YAW_L, -yaw_out)
        end
        hdg_ok = math.abs(hdg_err) < Config.YAW_THRESHOLD
    else
        rsOff(Config.SIDE_YAW_L)
        rsOff(Config.SIDE_YAW_R)
    end

    -- ── Forward speed ─────────────────────────────────────────
    -- Only thrust forward if heading is roughly correct
    if self.mode ~= Nav.MODE.HOVER and hdg_ok then
        local spd_out = self.pid_spd:compute(self.target_spd, s.speed, dt)
        if spd_out >= 0 then
            rsSet(Config.SIDE_THRUST_F, spd_out / Config.RS_MAX)
            rsOff(Config.SIDE_THRUST_B)
        else
            rsOff(Config.SIDE_THRUST_F)
            rsSet(Config.SIDE_THRUST_B, (-spd_out) / Config.RS_MAX)
        end
    else
        -- HOVER or not aligned: kill forward thrust
        rsOff(Config.SIDE_THRUST_F)
        rsOff(Config.SIDE_THRUST_B)
    end
end

-- ── All stop ──────────────────────────────────────────────────
function Nav:_allStop()
    rsOff(Config.SIDE_THRUST_F)
    rsOff(Config.SIDE_THRUST_B)
    rsOff(Config.SIDE_THRUST_U)
    rsOff(Config.SIDE_THRUST_D)
    rsOff(Config.SIDE_YAW_L)
    rsOff(Config.SIDE_YAW_R)
    self.pid_alt:reset()
    self.pid_spd:reset()
    self.pid_hdg:reset()
end

-- Manual direct redstone output (0-1 floats, called from main.lua)
function Nav:manualSet(fwd, back, up, down, yaw_l, yaw_r)
    rsSet(Config.SIDE_THRUST_F, fwd   or 0)
    rsSet(Config.SIDE_THRUST_B, back  or 0)
    rsSet(Config.SIDE_THRUST_U, up    or 0)
    rsSet(Config.SIDE_THRUST_D, down  or 0)
    rsSet(Config.SIDE_YAW_L,    yaw_l or 0)
    rsSet(Config.SIDE_YAW_R,    yaw_r or 0)
end

-- ── Status for GUI ────────────────────────────────────────────
function Nav:getStatus()
    local s = self.sensors
    return {
        mode        = self.mode,
        msg         = self.msg,
        -- sensor readings
        altitude    = s.altitude,
        speed       = s.speed,
        heading     = s.heading,
        pitch       = s.pitch,
        roll        = s.roll,
        -- velocity decomposition (from filtered DR)
        horiz_speed = s.horiz_speed,
        vert_speed  = s.vert_speed,
        vx          = s.vx,
        vz          = s.vz,
        -- dead reckoning position
        dr_x        = s.dr_x,
        dr_z        = s.dr_z,
        dr_dist     = s.dr_dist,
        -- meta
        elapsed     = self.elapsed,
        sensors     = s:status(),
        -- setpoints
        tgt_alt     = self.target_alt,
        tgt_spd     = self.target_spd,
        tgt_hdg     = self.target_hdg,
    }
end

return Nav
