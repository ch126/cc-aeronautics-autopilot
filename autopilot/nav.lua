-- =============================================================
--  autopilot/nav.lua

--

  -- 1. waypoint queue
  -- 2.  PID
  -- 3.
  -- 4.  main.lua
--
  -- Create:Aeronautics Helm API1.21.1
  -- helm.setThrottle(forward, up, right)   --  [-1,1]
  -- helm.setYawTarget(degrees)             --
  -- helm.getVelocity()                     --  {x,y,z}
  -- helm.getPosition()                     --  {x,y,z}
  -- helm.getYaw()                          --
  -- helm.getAssemblyInfo()                 --
  -- helm.disassemble()                     --
-- =============================================================

local Vec3     = require("autopilot.vec3")
local PIDMod   = require("autopilot.pid")
local Obstacle = require("autopilot.obstacle")
local Config   = require("autopilot.config")

local Nav = {}
Nav.__index = Nav


Nav.STATE = {
    IDLE      = "IDLE",
    NAVIGATING= "NAVIGATING",
    AVOIDING  = "AVOIDING",
    ARRIVED   = "ARRIVED",
    ERROR     = "ERROR",
}


-- Auto-detect the airship controller peripheral.
-- Create:Aeronautics registers its helm block as a CC peripheral.
-- The exact type name varies by version; we probe by checking for
-- the methods the autopilot needs (getPosition / setSpeed / etc.)
local function findHelm()
    -- 1. If config specifies a side/name, try that first
    if Config.HELM_SIDE then
        local p = peripheral.wrap(Config.HELM_SIDE)
        if p then return p, Config.HELM_SIDE end
    end

    -- 2. Try known type names via peripheral.find()
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

    -- 3. Scan ALL attached peripherals and pick the first one that has
    --    the methods we need (duck-typing).
    local required = { "getPosition", "setSpeed" }
    -- some versions use setThrottle instead
    local required_alt = { "getPosition", "setThrottle" }
    local required_alt2 = { "getPosition", "setSail" }
    for _, name in ipairs(peripheral.getNames()) do
        local p = peripheral.wrap(name)
        if p then
            local function hasAll(meths)
                for _, m in ipairs(meths) do
                    if type(p[m]) ~= "function" then return false end
                end
                return true
            end
            if hasAll(required) or hasAll(required_alt) or hasAll(required_alt2) then
                return p, name
            end
        end
    end

    return nil, nil
end

function Nav.new()
    local helm, helmName = findHelm()
    local radar = Config.HAS_RADAR
                  and peripheral.find(Config.RADAR_NAME)
                  or nil

    if not helm then
        -- List available peripherals to help user debug
        local names = peripheral.getNames()
        local list = #names > 0 and table.concat(names, ", ") or "(none)"
        error("[NAV] Airship controller peripheral not found!\n"
            .."Available peripherals: " .. list .. "\n"
            .."Steps to fix:\n"
            .."  1. Assemble the airship (right-click the Helm block)\n"
            .."  2. Place the computer ON the airship structure\n"
            .."  3. Connect computer to Helm with a Wired Modem + cable\n"
            .."     OR place computer directly adjacent to Helm\n"
            .."  4. If still failing, set Config.HELM_SIDE to the\n"
            .."     peripheral name shown above in config.lua")
    end

    -- Detect which throttle API this helm version exposes
    local helmAPI = {}
    if type(helm.setSpeed) == "function" then
        -- Create:Aeronautics newer API
        helmAPI.setThrottle = function(f, u, r)
            pcall(helm.setSpeed, f, u, r)
        end
    elseif type(helm.setThrottle) == "function" then
        helmAPI.setThrottle = function(f, u, r)
            pcall(helm.setThrottle, f, u, r)
        end
    elseif type(helm.setSail) == "function" then
        helmAPI.setThrottle = function(f, u, r)
            pcall(helm.setSail, f, u, r)
        end
    else
        helmAPI.setThrottle = function() end  -- no-op fallback
    end

    helmAPI.getPosition = helm.getPosition
    helmAPI.getVelocity = helm.getVelocity or function() return {x=0,y=0,z=0} end
    helmAPI.getYaw      = helm.getYaw      or function() return 0 end
    helmAPI.setYawTarget= helm.setYawTarget or function() end
    helmAPI._name       = helmName

    local pid3 = PIDMod.PID3.new(Config.PID_H, Config.PID_V)
    local pidY = PIDMod.PID.new(Config.PID_YAW)
    local obs  = Obstacle.new(radar)

    return setmetatable({
        helm       = helmAPI,
        pid3       = pid3,
        pid_yaw    = pidY,
        obstacle   = obs,


        waypoints  = {},
        wp_index   = 1,


        state      = Nav.STATE.IDLE,
        status_msg = "Standby",


        pos        = Vec3.new(0,0,0),
        velocity   = Vec3.new(0,0,0),
        yaw        = 0.0,
        tick       = 0,


        total_dist  = 0.0,
        elapsed_sec = 0.0,
    }, Nav)
end



  -- -
function Nav:addWaypoint(x, y, z)
    table.insert(self.waypoints, Vec3.new(x, y, z))
end

  -- -
function Nav:clearWaypoints()
    self.waypoints = {}
    self.wp_index  = 1
    self:_stop()
    self.state = Nav.STATE.IDLE
    self.status_msg = "Waypoints cleared"
end

  -- -
function Nav:start()
    if #self.waypoints == 0 then
        self.status_msg = "Error: no waypoints set"
        self.state = Nav.STATE.ERROR
        return
    end
    self.wp_index = 1
    self.pid3:reset()
    self.pid_yaw:reset()
    self.state      = Nav.STATE.NAVIGATING
    self.status_msg = string.format("Navigating to WP 1/%d", #self.waypoints)
end

  -- -
function Nav:stop()
    self:_stop()
    self.state      = Nav.STATE.IDLE
    self.status_msg = "Manually stopped"
end

  -- TICK_RATE
  -- -@param dt number
function Nav:update(dt)
    self.tick          = self.tick + 1
    self.elapsed_sec   = self.elapsed_sec + dt

    if self.state ~= Nav.STATE.NAVIGATING
    and self.state ~= Nav.STATE.AVOIDING then
        return
    end

  -- 1.
    self:_readState()

  -- 2.
    local target = self.waypoints[self.wp_index]
    if not target then
        self:_stop()
        self.state      = Nav.STATE.ARRIVED
        self.status_msg = "All waypoints reached!"
        return
    end

  -- 3.
    local dist = (self.pos - target):length()
    if dist <= Config.ARRIVAL_RADIUS then
        self.wp_index = self.wp_index + 1
        if self.wp_index > #self.waypoints then
            self:_stop()
            self.state      = Nav.STATE.ARRIVED
            self.status_msg = string.format("Arrived! ODO %.1f blk, time %.1f s",
                                            self.total_dist, self.elapsed_sec)
        else
            self.status_msg = string.format("WP reached, going %d/%d",
                                            self.wp_index, #self.waypoints)
            self.pid3:reset()
            self.pid_yaw:reset()
        end
        return
    end

  -- 4.
    local virtual_target, has_obs = self.obstacle:compute(
        self.pos, target, self.tick)

    if has_obs then
        self.state      = Nav.STATE.AVOIDING
        self.status_msg = string.format(
            "Avoiding -> WP %d/%d dist=%.1f",
            self.wp_index, #self.waypoints, dist)
    else
        self.state      = Nav.STATE.NAVIGATING
        self.status_msg = string.format(
            "Navigating WP %d/%d dist=%.1f blk",
            self.wp_index, #self.waypoints, dist)
    end

  -- 5. PID
    self:_controlStep(virtual_target, target, dt, dist)

  -- 6.
    if self._last_pos then
        self.total_dist = self.total_dist + (self.pos - self._last_pos):length()
    end
    self._last_pos = self.pos:clone()
end


function Nav:_readState()
    local ok, p = pcall(function() return self.helm.getPosition() end)
    if ok and p then
        self.pos = Vec3.new(p.x, p.y, p.z)
    end

    local ok2, v = pcall(function() return self.helm.getVelocity() end)
    if ok2 and v then
        self.velocity = Vec3.new(v.x, v.y, v.z)
    end

    local ok3, y = pcall(function() return self.helm.getYaw() end)
    if ok3 and y then
        self.yaw = y
    end
end

  -- PID
function Nav:_controlStep(vtarget, real_target, dt, dist)
    local cfg = Config
    local pos = self.pos


    local desired_yaw = pos:yawTo(vtarget)
    local yaw_err     = Vec3.normalizeAngle(desired_yaw - self.yaw)
    local yaw_out     = self.pid_yaw:compute(0, -yaw_err, dt)
  -- yaw_out > 0  < 0

  -- Create:Aeronautics setYaw
    pcall(self.helm.setYawTarget, desired_yaw)


    local abs_yaw_err = math.abs(yaw_err)
    local heading_ok  = abs_yaw_err < cfg.HEADING_THRESHOLD

  -- PID
    local ctrl = self.pid3:compute(
        { x = vtarget.x, y = vtarget.y, z = vtarget.z },
        { x = pos.x,     y = pos.y,     z = pos.z     },
        dt
    )

  -- /
    local yaw_rad = math.rad(self.yaw)
    local fwd  =  ctrl.x * math.sin(yaw_rad) + ctrl.z * (-math.cos(yaw_rad))
    local right=  ctrl.x * math.cos(yaw_rad) + ctrl.z * math.sin(yaw_rad)
    local up   =  ctrl.y


    local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
    fwd   = clamp(fwd,   -cfg.MAX_THROTTLE, cfg.MAX_THROTTLE)
    right = clamp(right, -cfg.MAX_THROTTLE, cfg.MAX_THROTTLE)
    up    = clamp(up,    -cfg.MAX_THROTTLE, cfg.MAX_THROTTLE)


    if not heading_ok then
        local factor = 1.0 - math.min(1.0, abs_yaw_err / 45.0)
        fwd   = fwd   * factor
        right = right * factor
    end


    -- helmAPI.setThrottle is already wrapped with pcall internally
    self.helm.setThrottle(fwd, up, right)
end


function Nav:_stop()
    self.helm.setThrottle(0, 0, 0)
    self.pid3:reset()
    self.pid_yaw:reset()
end


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
