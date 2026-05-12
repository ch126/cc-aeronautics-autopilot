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

    self.gim_p = findP(C.SENSOR_GIMBAL,   "gimbal_sensor")
    self.alt_p = findP(C.SENSOR_ALTITUDE,  "altitude_sensor")
    self.vel_p = findP(C.SENSOR_VELOCITY,  "velocity_sensor")

    -- ── 姿态角 (deg) ────────────────────────────────────────
    self.pitch = 0.0   -- 俯仰：+上仰
    self.roll  = 0.0   -- 横滚：+右倾
    self.yaw   = 0.0   -- 偏航：0=北 CW+

    -- ── 角速度 (deg/s)，有限差分 ──────────────────────────
    self.rate_p = 0.0  -- pitch rate
    self.rate_q = 0.0  -- roll  rate
    self.rate_r = 0.0  -- yaw   rate

    -- ── 高度 & 升降速 ──────────────────────────────────────
    self.altitude   = 0.0
    self.climb_rate = 0.0   -- EMA
    self._prev_alt  = nil

    -- ── 水平速度（标量 + DR方向） ──────────────────────────
    self.speed = 0.0

    -- ── 内部上一帧角度（用于角速度估算） ─────────────────
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
        local ok, v = pcall(self.gim_p.getAngles)
        if ok and type(v) == "table" then
            local new_pitch = tonumber(v[1] or v.pitch) or 0
            local new_roll  = tonumber(v[2] or v.roll)  or 0
            local new_yaw   = tonumber(v[3] or v.yaw)   or 0

            -- 角速度：有限差分 + EMA 平滑
            if self._init and dt and dt > 0 then
                self.rate_p = ema(self.rate_p, angDiff(new_pitch, self._prev.pitch)/dt, 0.4)
                self.rate_q = ema(self.rate_q, angDiff(new_roll,  self._prev.roll )/dt, 0.4)
                self.rate_r = ema(self.rate_r, angDiff(new_yaw,   self._prev.yaw  )/dt, 0.4)
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
        local ok, v = pcall(self.alt_p.getHeight)
        if ok and type(v) == "number" then
            if self._prev_alt and dt and dt > 0 then
                local raw_climb = (v - self._prev_alt) / dt
                self.climb_rate = ema(self.climb_rate, raw_climb, 0.3)
            end
            self._prev_alt = v
            self.altitude  = v
        end
    end

    -- ── 速度 ───────────────────────────────────────────────
    if self.vel_p then
        local ok, v = pcall(self.vel_p.getVelocity)
        if ok and type(v) == "number" then
            self.speed = math.abs(v)
        end
    end

    self._init = true
end

function IMU:status()
    local function yn(p) return p and "OK" or "--" end
    return string.format("gim:%s alt:%s vel:%s",
        yn(self.gim_p), yn(self.alt_p), yn(self.vel_p))
end

return IMU
