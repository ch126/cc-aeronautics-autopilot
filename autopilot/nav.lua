-- =============================================================
--  autopilot/nav.lua
--
--  Control modes (auto-detected by peripheral capability):
--
--  MODE A: Direct Velocity  (preferred)
--    helm.setVelocity(vx, vy, vz)  +  helm.setYawTarget(deg)
--    Nav computes desired world-space velocity from position error.
--    No PID tuning required.
--
--  MODE B: PID Throttle  (fallback)
--    helm.setThrottle / setSpeed / setSail  (local-frame throttle)
--    PID drives position error -> throttle output.
--
--  Position / velocity / yaw are always read from the helm peripheral.
-- =============================================================

local Vec3     = require("autopilot.vec3")
local PIDMod   = require("autopilot.pid")
local Obstacle = require("autopilot.obstacle")
local Config   = require("autopilot.config")

local Nav = {}
Nav.__index = Nav

Nav.STATE = {
    IDLE       = "IDLE",
    NAVIGATING = "NAVIGATING",
    AVOIDING   = "AVOIDING",
    ARRIVED    = "ARRIVED",
    ERROR      = "ERROR",
}

-- ── Peripheral auto-detect ────────────────────────────────────
local function findHelm()
    if Config.HELM_SIDE then
        local p = peripheral.wrap(Config.HELM_SIDE)
        if p then return p, Config.HELM_SIDE end
    end

    local knownTypes = {
        "create_aeronautics:airship_helm",
        "create_aeronautics:tilt_airship_helm",
        "create_aeronautics:helm",
        "airshipHelm",
        "airship_helm",
        "Aeronautics_AirshipHelm",
    }
    for _, t in ipairs(knownTypes) do
        local p = peripheral.find(t)
        if p then return p, t end
    end

    -- Duck-type scan: need at least getPosition + some control method
    local needs_pos   = { "getPosition" }
    local ctrl_sets   = {
        { "setVelocity" },
        { "setSpeed" },
        { "setThrottle" },
        { "setSail" },
    }
    for _, name in ipairs(peripheral.getNames()) do
        local p = peripheral.wrap(name)
        if p and type(p.getPosition) == "function" then
            for _, cs in ipairs(ctrl_sets) do
                local ok = true
                for _, m in ipairs(cs) do
                    if type(p[m]) ~= "function" then ok = false; break end
                end
                if ok then return p, name end
            end
        end
    end

    return nil, nil
end

-- ── Constructor ───────────────────────────────────────────────
function Nav.new()
    local helm, helmName = findHelm()

    if not helm then
        local names = peripheral.getNames()
        local list  = #names > 0 and table.concat(names, ", ") or "(none)"
        error("[NAV] Airship controller not found!\n"
            .. "Available peripherals: " .. list .. "\n"
            .. "Fix:\n"
            .. "  1. Assemble ship (right-click Helm)\n"
            .. "  2. Place computer ON ship structure\n"
            .. "  3. Connect via Wired Modem + cable, or place adjacent\n"
            .. "  4. Set Config.HELM_SIDE in config.lua if needed")
    end

    -- Determine control mode
    local ctrlMode   -- "velocity" | "throttle"
    local helmAPI    = {}

    if type(helm.setVelocity) == "function" then
        -- ── MODE A: Direct velocity ────────────────────────────
        ctrlMode = "velocity"
        helmAPI.setVelocity = function(vx, vy, vz)
            pcall(helm.setVelocity, vx, vy, vz)
        end
        helmAPI.setThrottle = function() end  -- no-op
    elseif type(helm.setSpeed) == "function" then
        ctrlMode = "throttle"
        helmAPI.setThrottle = function(f, u, r)
            pcall(helm.setSpeed, f, u, r)
        end
    elseif type(helm.setThrottle) == "function" then
        ctrlMode = "throttle"
        helmAPI.setThrottle = function(f, u, r)
            pcall(helm.setThrottle, f, u, r)
        end
    elseif type(helm.setSail) == "function" then
        ctrlMode = "throttle"
        helmAPI.setThrottle = function(f, u, r)
            pcall(helm.setSail, f, u, r)
        end
    else
        ctrlMode = "none"
        helmAPI.setThrottle = function() end
    end

    helmAPI.getPosition  = helm.getPosition
    helmAPI.getVelocity  = helm.getVelocity  or function() return {x=0,y=0,z=0} end
    helmAPI.getYaw       = helm.getYaw       or function() return 0 end
    helmAPI.setYawTarget = helm.setYawTarget or function() end
    helmAPI._name        = helmName
    helmAPI._mode        = ctrlMode

    local pid3 = PIDMod.PID3.new(Config.PID_H, Config.PID_V)
    local pidY = PIDMod.PID.new(Config.PID_YAW)
    local obs  = Obstacle.new(
        Config.HAS_RADAR and peripheral.find(Config.RADAR_NAME) or nil)

    return setmetatable({
        helm        = helmAPI,
        ctrl_mode   = ctrlMode,
        pid3        = pid3,
        pid_yaw     = pidY,
        obstacle    = obs,

        waypoints   = {},
        wp_index    = 1,

        state       = Nav.STATE.IDLE,
        status_msg  = "Standby [mode:" .. ctrlMode .. "]",

        pos         = Vec3.new(0,0,0),
        velocity    = Vec3.new(0,0,0),
        yaw         = 0.0,
        tick        = 0,

        total_dist  = 0.0,
        elapsed_sec = 0.0,
    }, Nav)
end

-- ── Waypoint management ───────────────────────────────────────
function Nav:addWaypoint(x, y, z)
    table.insert(self.waypoints, Vec3.new(x, y, z))
end

function Nav:clearWaypoints()
    self.waypoints = {}
    self.wp_index  = 1
    self:_stop()
    self.state      = Nav.STATE.IDLE
    self.status_msg = "Waypoints cleared"
end

function Nav:start()
    if #self.waypoints == 0 then
        self.status_msg = "Error: no waypoints"
        self.state      = Nav.STATE.ERROR
        return
    end
    self.wp_index = 1
    self.pid3:reset()
    self.pid_yaw:reset()
    self.state      = Nav.STATE.NAVIGATING
    self.status_msg = string.format("Navigating WP 1/%d [%s]",
        #self.waypoints, self.ctrl_mode)
end

function Nav:stop()
    self:_stop()
    self.state      = Nav.STATE.IDLE
    self.status_msg = "Stopped"
end

-- ── Update (called every tick) ────────────────────────────────
function Nav:update(dt)
    self.tick        = self.tick + 1
    self.elapsed_sec = self.elapsed_sec + dt

    if self.state ~= Nav.STATE.NAVIGATING
    and self.state ~= Nav.STATE.AVOIDING then
        return
    end

    self:_readState()

    local target = self.waypoints[self.wp_index]
    if not target then
        self:_stop()
        self.state      = Nav.STATE.ARRIVED
        self.status_msg = "All waypoints reached!"
        return
    end

    local dist = (self.pos - target):length()
    if dist <= Config.ARRIVAL_RADIUS then
        self.wp_index = self.wp_index + 1
        if self.wp_index > #self.waypoints then
            self:_stop()
            self.state      = Nav.STATE.ARRIVED
            self.status_msg = string.format("Arrived! ODO=%.1f blk  t=%.1f s",
                self.total_dist, self.elapsed_sec)
        else
            self.status_msg = string.format("WP reached, going %d/%d",
                self.wp_index, #self.waypoints)
            self.pid3:reset(); self.pid_yaw:reset()
        end
        return
    end

    local vtarget, has_obs = self.obstacle:compute(self.pos, target, self.tick)
    if has_obs then
        self.state      = Nav.STATE.AVOIDING
        self.status_msg = string.format("Avoiding -> WP%d dist=%.1f", self.wp_index, dist)
    else
        self.state      = Nav.STATE.NAVIGATING
        self.status_msg = string.format("WP%d/%d dist=%.1f [%s]",
            self.wp_index, #self.waypoints, dist, self.ctrl_mode)
    end

    if self.ctrl_mode == "velocity" then
        self:_controlVelocity(vtarget, dt)
    else
        self:_controlThrottle(vtarget, dt)
    end

    if self._last_pos then
        self.total_dist = self.total_dist + (self.pos - self._last_pos):length()
    end
    self._last_pos = self.pos:clone()
end

-- ── State read ────────────────────────────────────────────────
function Nav:_readState()
    local ok, p = pcall(self.helm.getPosition)
    if ok and p then self.pos = Vec3.new(p.x, p.y, p.z) end

    local ok2, v = pcall(self.helm.getVelocity)
    if ok2 and v then self.velocity = Vec3.new(v.x, v.y, v.z) end

    local ok3, y = pcall(self.helm.getYaw)
    if ok3 and y then self.yaw = y end
end

-- ── MODE A: Direct velocity control ──────────────────────────
-- Computes desired world-space velocity proportional to position error.
-- No PID needed — just a P-gain on position, clamped to max speed.
function Nav:_controlVelocity(vtarget, dt)
    local cfg    = Config
    local err    = vtarget - self.pos   -- world-space error vector

    -- Proportional: desired_velocity = Kp * error, clamped to MAX_SPEED
    local Kp       = cfg.VEL_KP or 0.5
    local max_spd  = cfg.MAX_SPEED or 10.0
    local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end

    local vx = clamp(err.x * Kp, -max_spd, max_spd)
    local vy = clamp(err.y * Kp, -max_spd, max_spd)
    local vz = clamp(err.z * Kp, -max_spd, max_spd)

    -- Slow down near arrival
    local dist = err:length()
    if dist < cfg.ARRIVAL_RADIUS * 3 then
        local factor = dist / (cfg.ARRIVAL_RADIUS * 3)
        vx = vx * factor
        vy = vy * factor
        vz = vz * factor
    end

    self.helm.setVelocity(vx, vy, vz)

    -- Yaw: face direction of travel (or target if nearly arrived)
    local desired_yaw = self.pos:yawTo(vtarget)
    pcall(self.helm.setYawTarget, desired_yaw)
end

-- ── MODE B: PID throttle control ─────────────────────────────
function Nav:_controlThrottle(vtarget, dt)
    local cfg = Config
    local pos = self.pos

    local desired_yaw = pos:yawTo(vtarget)
    local yaw_err     = Vec3.normalizeAngle(desired_yaw - self.yaw)
    local abs_yaw_err = math.abs(yaw_err)
    local heading_ok  = abs_yaw_err < cfg.HEADING_THRESHOLD

    pcall(self.helm.setYawTarget, desired_yaw)

    local ctrl = self.pid3:compute(
        { x=vtarget.x, y=vtarget.y, z=vtarget.z },
        { x=pos.x,     y=pos.y,     z=pos.z     },
        dt)

    local yaw_rad = math.rad(self.yaw)
    local fwd   =  ctrl.x * math.sin(yaw_rad) + ctrl.z * (-math.cos(yaw_rad))
    local right =  ctrl.x * math.cos(yaw_rad) + ctrl.z * math.sin(yaw_rad)
    local up    =  ctrl.y

    local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
    fwd   = clamp(fwd,   -cfg.MAX_THROTTLE, cfg.MAX_THROTTLE)
    right = clamp(right, -cfg.MAX_THROTTLE, cfg.MAX_THROTTLE)
    up    = clamp(up,    -cfg.MAX_THROTTLE, cfg.MAX_THROTTLE)

    if not heading_ok then
        local factor = 1.0 - math.min(1.0, abs_yaw_err / 45.0)
        fwd   = fwd   * factor
        right = right * factor
    end

    self.helm.setThrottle(fwd, up, right)
end

-- ── Stop ─────────────────────────────────────────────────────
function Nav:_stop()
    if self.ctrl_mode == "velocity" then
        pcall(self.helm.setVelocity, 0, 0, 0)
    else
        pcall(self.helm.setThrottle, 0, 0, 0)
    end
    self.pid3:reset()
    self.pid_yaw:reset()
end

-- ── Status ────────────────────────────────────────────────────
function Nav:getStatus()
    return {
        state      = self.state,
        msg        = self.status_msg,
        pos        = self.pos,
        yaw        = self.yaw,
        velocity   = self.velocity,
        wp_current = self.wp_index,
        wp_total   = #self.waypoints,
        target     = self.waypoints[self.wp_index],
        dist       = self.waypoints[self.wp_index]
                     and (self.pos - self.waypoints[self.wp_index]):length()
                     or  0,
        total_dist = self.total_dist,
        elapsed    = self.elapsed_sec,
    }
end

return Nav
