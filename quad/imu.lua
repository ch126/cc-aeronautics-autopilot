-- =============================================================
--  quad/imu.lua
--  IMU 传感器封装
--
--  提供：
--    angles.pitch / roll / yaw  (deg, 右手系)
--    rates.p / q / r            (deg/s, 有限差分估算)
--    altitude                   (blocks, 绝对高度)
--    climb_rate                 (blocks/s, EMA估算)
--    速度向量 vx/vz             (m/s, DR推算)
-- =============================================================

local C = dofile("/quad/config.lua")

local function findP(name, type_)
    if name then
        local p = peripheral.wrap(name)
        if p then return p end
    end
    return peripheral.find(type_)
end

local IMU = {}
IMU.__index = IMU

function IMU.new()
    local self = setmetatable({}, IMU)

    self.gim_p   = findP(C.SENSOR_GIMBAL,   "gimbal_sensor")
    self.alt_p   = findP(C.SENSOR_ALTITUDE,  "altitude_sensor")
    -- 两个速度传感器，分别朝 X 和 Z 轴
    local all_vel = {}
    peripheral.find("velocity_sensor", function(name, p) all_vel[#all_vel+1] = {name=name, p=p} end)
    if C.SENSOR_VEL_X then
        self.vel_x_p = peripheral.wrap(C.SENSOR_VEL_X)
    else
        self.vel_x_p = all_vel[1] and all_vel[1].p or nil
    end
    if C.SENSOR_VEL_Z then
        self.vel_z_p = peripheral.wrap(C.SENSOR_VEL_Z)
    else
        self.vel_z_p = all_vel[2] and all_vel[2].p or nil
    end

    -- ── 姿态角 (deg) ────────────────────────────────────────
    self.pitch = 0.0
    self.roll  = 0.0
    self.yaw   = 0.0

    -- ── 角速度 (deg/s) ────────────────────────────────────
    self.rate_p = 0.0
    self.rate_q = 0.0
    self.rate_r = 0.0

    -- ── 高度 & 升降速 ──────────────────────────────────────
    self.altitude   = 0.0
    self.climb_rate = 0.0
    self._prev_alt  = nil

    -- ── 水平速度向量 (m/s, 世界坐标系) ───────────────────
    self.vx    = 0.0   -- North+
    self.vz    = 0.0   -- East+
    self.speed = 0.0   -- 标量

    -- ── GPS 位置 ──────────────────────────────────────────
    self.x     = nil   -- 世界 X 坐标
    self.y     = nil   -- 世界 Y 坐标
    self.z     = nil   -- 世界 Z 坐标

    -- ── 内部 ─────────────────────────────────────────────
    self._prev = {pitch=0, roll=0, yaw=0}
    self._init = false

    return self
end

-- 最短角度差（处理 360/0 边界）
local function angDiff(a, b)
    local d = (a - b) % 360
    if d > 180 then d = d - 360 end
    return d
end

-- EMA
local function ema(old, new, alpha)
    return old + alpha * (new - old)
end

function IMU:read(dt)
    -- ── 姿态角 ────────────────────────────────────────────
    if self.gim_p then
        local ok, v = pcall(function() return self.gim_p.getAngles() end)
        if ok and type(v) == "table" then
            local new_pitch = tonumber(v[1] or v.pitch) or 0
            local new_roll  = tonumber(v[2] or v.roll)  or 0
            local new_yaw   = tonumber(v[3] or v.yaw)   or 0

            -- 角速度：有限差分 + EMA 平滑（alpha 0.6 = 快速响应）
            if self._init and dt and dt > 0 then
                self.rate_p = ema(self.rate_p, angDiff(new_pitch, self._prev.pitch)/dt, 0.6)
                self.rate_q = ema(self.rate_q, angDiff(new_roll,  self._prev.roll )/dt, 0.6)
                self.rate_r = ema(self.rate_r, angDiff(new_yaw,   self._prev.yaw  )/dt, 0.6)
            end

            self._prev.pitch = new_pitch
            self._prev.roll  = new_roll
            self._prev.yaw   = new_yaw

            self.pitch = new_pitch
            self.roll  = new_roll
            self.yaw   = new_yaw
        end
    end

    -- ── 高度 & 升降速 ──────────────────────────────────────
    if self.alt_p then
        local ok, v = pcall(function() return self.alt_p.getHeight() end)
        if ok and type(v) == "number" then
            if self._prev_alt and dt and dt > 0 then
                local raw_climb = (v - self._prev_alt) / dt
                self.climb_rate = ema(self.climb_rate, raw_climb, 0.5)  -- 0.5 快速跟踪
            end
            self._prev_alt = v
            self.altitude  = v
        end
    end

    -- ── 速度向量 ──────────────────────────────────────────
    -- vel_x_p：朝 X 轴安装，getVelocity() 返回 X 方向速度 (m/s)
    -- vel_z_p：朝 Z 轴安装，getVelocity() 返回 Z 方向速度 (m/s)
    if self.vel_x_p then
        local ok, v = pcall(function() return self.vel_x_p.getVelocity() end)
        if ok and type(v) == "number" then
            self.vx = v
        end
    end
    if self.vel_z_p then
        local ok, v = pcall(function() return self.vel_z_p.getVelocity() end)
        if ok and type(v) == "number" then
            self.vz = -v  -- Z轴传感器方向相反，取反
        end
    end
    self.speed = math.sqrt(self.vx^2 + self.vz^2)

    self._init = true
    return self
end

-- 单独调用，放在慢速循环（1~2Hz），避免阻塞控制环
function IMU:readGPS()
    local gx, gy, gz = gps.locate(2)
    if gx then
        -- gps 返回的 x/z 与飞控坐标系相反，对调修正
        self.x = gz
        self.y = gy
        self.z = gx
    end
end

function IMU:status()
    local function yn(p) return p and "OK" or "--" end
    return string.format("gim:%s alt:%s vx:%s vz:%s",
        yn(self.gim_p), yn(self.alt_p), yn(self.vel_x_p), yn(self.vel_z_p))
end

return IMU
