-- =============================================================
--  car/nav.lua
--  自动驾驶汽车导航控制器
--
--  导航逻辑：
--    1. 计算到目标的方向角 (bearing)
--    2. PID 转向：让车头对准 bearing
--    3. 对准后 PID 油门：以 target_speed 行驶
--    4. 接近目标时减速，到达后停车
--
--  航点队列：支持多航点顺序执行
-- =============================================================

local PIDMod  = dofile("/autopilot/pid.lua")   -- 复用飞控 PID
local Sensors = dofile("/car/sensors.lua")
local Config  = dofile("/car/config.lua")

local Nav = {}
Nav.__index = Nav

Nav.MODE = {
    IDLE    = "IDLE",
    DRIVE   = "DRIVE",   -- 前往单个目标
    ROUTE   = "ROUTE",   -- 执行航点队列
    MANUAL  = "MANUAL",
}

-- ── 红石输出 ──────────────────────────────────────────────────
local function rsSet(side, v)
    if not side then return end
    rs.setAnalogOutput(side, math.floor(math.max(0, math.min(Config.RS_MAX, v)) + 0.5))
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

local function softScale(err, zone)
    if zone <= 0 then return 1 end
    return clamp(math.abs(err) / zone, 0, 1)
end

-- ── 构造 ──────────────────────────────────────────────────────
function Nav.new()
    local sens = Sensors.new()
    sens:read(0)

    local self = setmetatable({
        sensors     = sens,
        pid_spd     = PIDMod.PID.new(Config.PID_SPD),
        pid_steer   = PIDMod.PID.new(Config.PID_STEER),

        mode        = Nav.MODE.IDLE,
        msg         = "Standby",

        -- 当前目标（世界坐标）
        target_x    = nil,
        target_z    = nil,
        target_spd  = Config.MAX_SPEED,

        -- 航点队列 [{x, z, speed}]
        waypoints   = {},
        wp_index    = 1,

        elapsed     = 0.0,
    }, Nav)
    return self
end

-- ── 设置单个目标 ──────────────────────────────────────────────
function Nav:driveTo(x, z, speed)
    self.target_x   = x
    self.target_z   = z
    self.target_spd = speed or Config.MAX_SPEED
    self.mode       = Nav.MODE.DRIVE
    self.pid_spd:reset()
    self.pid_steer:reset()
    self.msg = string.format("Drive to X=%.0f Z=%.0f", x, z)
end

-- ── 航点队列 ──────────────────────────────────────────────────
function Nav:setRoute(wps)
    self.waypoints  = wps   -- [{x, z, speed}]
    self.wp_index   = 1
    if #wps > 0 then
        local wp = wps[1]
        self:driveTo(wp.x, wp.z, wp.speed)
        self.mode = Nav.MODE.ROUTE
        self.msg = string.format("Route: %d waypoints", #wps)
    end
end

function Nav:addWaypoint(x, z, speed)
    table.insert(self.waypoints, {x=x, z=z, speed=speed or Config.MAX_SPEED})
    if self.mode == Nav.MODE.IDLE then
        self:setRoute(self.waypoints)
    end
end

function Nav:clearRoute()
    self.waypoints = {}
    self.wp_index  = 1
    self:stop()
end

-- ── 停车 ──────────────────────────────────────────────────────
function Nav:stop()
    rsOff(Config.SIDE_THROTTLE_F)
    rsOff(Config.SIDE_THROTTLE_B)
    rsOff(Config.SIDE_STEER_L)
    rsOff(Config.SIDE_STEER_R)
    self.pid_spd:reset()
    self.pid_steer:reset()
    self.mode = Nav.MODE.IDLE
    self.msg  = "Stopped"
end

-- ── 主更新循环 ────────────────────────────────────────────────
function Nav:update(dt)
    self.elapsed = self.elapsed + dt
    self.sensors:read(dt)

    if self.mode == Nav.MODE.IDLE or self.mode == Nav.MODE.MANUAL then
        return
    end

    if self.target_x == nil then return end

    local cx, cz = self.sensors:pos()

    -- 到目标的距离和方向
    local dx   = self.target_x - cx
    local dz   = self.target_z - cz
    local dist = math.sqrt(dx*dx + dz*dz)

    -- 到达判定
    if dist <= Config.ARRIVAL_RADIUS then
        if self.mode == Nav.MODE.ROUTE then
            -- 前进到下一个航点
            self.wp_index = self.wp_index + 1
            if self.wp_index <= #self.waypoints then
                local wp = self.waypoints[self.wp_index]
                self:driveTo(wp.x, wp.z, wp.speed)
                self.mode = Nav.MODE.ROUTE
                self.msg = string.format("WP %d/%d -> X=%.0f Z=%.0f",
                    self.wp_index, #self.waypoints, wp.x, wp.z)
            else
                self:stop()
                self.msg = "Route complete!"
            end
        else
            self:stop()
            self.msg = string.format("Arrived! (err ~%.1f blk)", dist)
        end
        return
    end

    -- 目标方位角：atan2(dx, -dz)，北=0, 东=90
    local bearing = math.deg(math.atan(dx, -dz)) % 360

    -- 接近目标时减速
    local speed_limit = self.target_spd
    if dist < Config.BRAKE_ZONE then
        speed_limit = math.max(Config.MIN_SPEED,
            self.target_spd * (dist / Config.BRAKE_ZONE))
    end

    self.msg = string.format("WP%d dist=%.0f bear=%.0f spd=%.1f",
        self.wp_index, dist, bearing, speed_limit)

    self:_controlStep(dt, bearing, speed_limit)
end

-- ── 控制步骤 ──────────────────────────────────────────────────
function Nav:_controlStep(dt, bearing, speed_limit)
    local s = self.sensors

    -- 航向误差（-180..180）
    local hdg_err = normalizeAngle(bearing - s.heading)

    -- ── 转向 ──────────────────────────────────────────────────
    local steer = self.pid_steer:compute(0, -hdg_err, dt)
    local steer_scale = softScale(hdg_err, 30.0)
    steer = clamp(steer * steer_scale, -Config.RS_MAX, Config.RS_MAX)

    if steer >= 0 then
        rsSet(Config.SIDE_STEER_R, steer)
        rsOff(Config.SIDE_STEER_L)
    else
        rsOff(Config.SIDE_STEER_R)
        rsSet(Config.SIDE_STEER_L, -steer)
    end

    -- ── 油门（航向大偏差时减速） ──────────────────────────────
    -- 偏差超过 60° 时停车原地转向
    local abs_hdg_err = math.abs(hdg_err)
    local throttle_factor = 1.0
    if abs_hdg_err > 60 then
        throttle_factor = 0.0   -- 停车转向
    elseif abs_hdg_err > Config.STEER_THRESHOLD then
        -- 偏差 3°~60° 线性降速
        throttle_factor = 1.0 - (abs_hdg_err - Config.STEER_THRESHOLD)
                              / (60 - Config.STEER_THRESHOLD)
    end

    local throttle = self.pid_spd:compute(
        speed_limit * throttle_factor, s.speed, dt)
    local spd_scale = softScale(speed_limit - s.speed, Config.MAX_SPEED * 0.3)
    throttle = clamp(throttle * spd_scale, -Config.RS_MAX, Config.RS_MAX)

    if throttle >= 0 then
        rsSet(Config.SIDE_THROTTLE_F, throttle)
        rsOff(Config.SIDE_THROTTLE_B)
    else
        rsOff(Config.SIDE_THROTTLE_F)
        rsSet(Config.SIDE_THROTTLE_B, -throttle)   -- 刹车/倒车
    end
end

-- ── 获取状态 ──────────────────────────────────────────────────
function Nav:getStatus()
    local s  = self.sensors
    local cx, cz = s:pos()
    return {
        mode      = self.mode,
        msg       = self.msg,
        x         = cx,
        z         = cz,
        heading   = s.heading,
        speed     = s.speed,
        dr_dist   = s.dr_dist,
        gps_ok    = s.gps_ok,
        sensors   = s:status(),
        elapsed   = self.elapsed,
        wp_total  = #self.waypoints,
        wp_index  = self.wp_index,
        target_x  = self.target_x,
        target_z  = self.target_z,
    }
end

return Nav
