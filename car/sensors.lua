-- =============================================================
--  car/sensors.lua
--  汽车传感器封装
--
--  支持两种定位方式：
--    1. GPS 外设（精确）: getPosition() -> {x, y, z}
--    2. 死区推算 DR（无 GPS 时降级）: 用速度+航向积分
-- =============================================================

local Config = dofile("/car/config.lua")

local function findP(name, type_)
    if name then
        local p = peripheral.wrap(name)
        if p then return p end
    end
    return peripheral.find(type_)
end

local function hdgDelta(target, current)
    local d = (target - current) % 360
    if d > 180 then d = d - 360 end
    return d
end

local Sensors = {}
Sensors.__index = Sensors

function Sensors.new()
    local self = setmetatable({}, Sensors)

    self.nav_p = findP(Config.SENSOR_NAV,      "navigation_table")
    self.vel_p = findP(Config.SENSOR_VELOCITY,  "velocity_sensor")
    self.gps_p = findP(Config.SENSOR_GPS,       "gps_sensor")

    -- 当前读数
    self.speed    = 0.0
    self.heading  = 0.0   -- 0=北, 90=东

    -- GPS 坐标（有 GPS 时使用）
    self.gps_ok   = false
    self.x        = 0.0
    self.z        = 0.0

    -- 死区推算位置（GPS 不可用时）
    self.dr_x     = 0.0
    self.dr_z     = 0.0
    self.dr_dist  = 0.0

    -- DR 原点（用于相对坐标计算）
    self.origin_x = 0.0
    self.origin_z = 0.0
    self.origin_set = false

    -- EMA 滤波
    self._filt_speed = 0.0
    self._filt_hdg   = 0.0
    self._init       = false

    return self
end

function Sensors:setOrigin()
    self.origin_x   = self.x
    self.origin_z   = self.z
    self.dr_x       = 0.0
    self.dr_z       = 0.0
    self.dr_dist    = 0.0
    self.origin_set = true
end

function Sensors:read(dt)
    -- 速度
    local raw_spd = self.speed
    if self.vel_p then
        local ok, v = pcall(self.vel_p.getVelocity)
        if ok and type(v) == "number" then
            raw_spd = math.abs(v)
            self.speed = raw_spd
        end
    end

    -- 航向
    local raw_hdg = self.heading
    if self.nav_p then
        local ok, v = pcall(self.nav_p.getRelativeAngle)
        if ok and type(v) == "number" then
            raw_hdg = v % 360
            self.heading = raw_hdg
        end
    end

    -- GPS（优先用精确坐标）
    if self.gps_p then
        local ok, pos = pcall(self.gps_p.getPosition)
        if ok and type(pos) == "table" then
            self.x      = tonumber(pos.x or pos[1]) or self.x
            self.z      = tonumber(pos.z or pos[3]) or self.z
            self.gps_ok = true
        else
            self.gps_ok = false
        end
    end

    -- EMA 初始化
    if not self._init then
        self._filt_speed = raw_spd
        self._filt_hdg   = raw_hdg
        self._init = true
        if not self.origin_set then self:setOrigin() end
    end

    -- EMA 滤波
    self._filt_speed = self._filt_speed
        + Config.DR_EMA_SPEED * (raw_spd - self._filt_speed)
    local dh = hdgDelta(raw_hdg, self._filt_hdg)
    self._filt_hdg = (self._filt_hdg + Config.DR_EMA_HDG * dh) % 360

    -- 死区推算（GPS 不可用时作为主要定位，有 GPS 也同步跑用于对比）
    if dt and dt > 0 and dt < 1.0
       and self._filt_speed >= Config.DR_MIN_SPEED then
        local hr  = math.rad(self._filt_hdg)
        local dx  =  self._filt_speed * math.sin(hr) * dt
        local dz  = -self._filt_speed * math.cos(hr) * dt
        self.dr_x    = self.dr_x + dx
        self.dr_z    = self.dr_z + dz
        self.dr_dist = self.dr_dist + math.sqrt(dx*dx + dz*dz)
        -- 无 GPS 时用 DR 更新世界坐标
        if not self.gps_ok then
            self.x = self.origin_x + self.dr_x
            self.z = self.origin_z + self.dr_z
        end
    end
end

-- 返回当前位置（GPS 优先，否则 DR）
function Sensors:pos()
    return self.x, self.z
end

function Sensors:status()
    local function yn(p) return p and "OK" or "--" end
    return string.format("nav:%s vel:%s gps:%s %s",
        yn(self.nav_p), yn(self.vel_p), yn(self.gps_p),
        self.gps_ok and "(GPS)" or "(DR)")
end

return Sensors
