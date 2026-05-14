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
    self.nav_p   = findP(C.SENSOR_NAV,       "navigation_table")
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
    self._vx_bias = 0.0  -- 速度传感器零点偏置（arm时标定）
    self._vz_bias = 0.0

    -- ── GPS 位置 ──────────────────────────────────────────
    self.x     = 0.0   -- 相对起飞点 X（航位推算）
    self.y     = nil   -- 世界 Y 坐标（暂不使用）
    self.z     = 0.0   -- 相对起飞点 Z（航位推算）

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
                self.climb_rate = ema(self.climb_rate, raw_climb, 0.25)  -- 0.25 平滑跟踪，减少噪声
            end
            self._prev_alt = v
            self.altitude  = v
        end
    end

    -- ── 速度向量 ──────────────────────────────────────────
    if self.vel_x_p then
        local ok, v = pcall(function() return self.vel_x_p.getVelocity() end)
        if ok and type(v) == "number" then
            self.vx = v - self._vx_bias
        end
    end
    if self.vel_z_p then
        local ok, v = pcall(function() return self.vel_z_p.getVelocity() end)
        if ok and type(v) == "number" then
            self.vz = -(v - self._vz_bias)
        end
    end
    self.speed = math.sqrt(self.vx^2 + self.vz^2)

    -- ── 飞行中自适应偏置估计 ───────────────────────────────
    -- 当姿态接近水平（pitch/roll < 2°）且速度很小时，
    -- 认为剩余速度读数全是偏置，缓慢更新之
    local abs_pitch = math.abs(self.pitch)
    local abs_roll  = math.abs(self.roll)
    if self._init and abs_pitch < 2.0 and abs_roll < 2.0 and self.speed < 0.3 then
        -- 极慢速追踪（约10秒时间常数），不影响正常飞行
        local BIAS_ALPHA = 0.002
        if self.vel_x_p then
            local ok, v = pcall(function() return self.vel_x_p.getVelocity() end)
            if ok and type(v) == "number" then
                self._vx_bias = self._vx_bias + BIAS_ALPHA * (v - self._vx_bias)
            end
        end
        if self.vel_z_p then
            local ok, v = pcall(function() return self.vel_z_p.getVelocity() end)
            if ok and type(v) == "number" then
                self._vz_bias = self._vz_bias + BIAS_ALPHA * (v - self._vz_bias)
            end
        end
    end

    -- ── 航位推算：速度积分得相对位置 ──────────────────────
    if self._init and dt and dt > 0 then
        self.x = self.x + self.vx * dt
        self.z = self.z + self.vz * dt
    end

    -- ── 导航台 Yaw + 位置修正（互补滤波）─────────────────────
    if self.nav_p and C.NAV_BEACON_BEARING then
        local ok, rel = pcall(function() return self.nav_p.getRelativeAngle() end)
        if ok and type(rel) == "number" then
            -- ① Yaw 修正：绝对朝向 = 信标方位角 - 相对角
            local nav_yaw = (C.NAV_BEACON_BEARING - rel) % 360
            local alpha = C.NAV_YAW_ALPHA or 0.98
            local diff = (nav_yaw - self.yaw) % 360
            if diff > 180 then diff = diff - 360 end
            self.yaw = (self.yaw + (1.0 - alpha) * diff) % 360

            -- ② 位置修正：利用方位角 + 已知信标坐标，修正DR漂移
            -- 飞机到信标的绝对方位角（世界系，北=0°）
            if C.NAV_BEACON_X and C.NAV_BEACON_Z then
                local abs_bearing_rad = math.rad(nav_yaw)
                -- 从当前DR位置出发，计算预期方位角
                local dbx = C.NAV_BEACON_X - self.x
                local dbz = C.NAV_BEACON_Z - self.z
                local dist = math.sqrt(dbx*dbx + dbz*dbz)
                if dist > 0.5 then
                    -- 预期方位角（DR计算）
                    local expected_bearing_rad = math.atan(dbx, dbz)  -- atan2(X,Z) = 北偏东
                    -- 实际方位角（导航台测量）
                    local measured_bearing_rad = abs_bearing_rad
                    -- 方位角差 → 垂直于视线方向的位置误差
                    local bearing_err = measured_bearing_rad - expected_bearing_rad
                    -- 归一化到 -π..π
                    while bearing_err >  math.pi do bearing_err = bearing_err - 2*math.pi end
                    while bearing_err < -math.pi do bearing_err = bearing_err + 2*math.pi end
                    -- 把方位角误差转换成横向位置修正
                    -- 横向偏移 ≈ dist * sin(bearing_err)
                    local lateral_err = dist * math.sin(bearing_err)
                    -- 横向方向（垂直于信标方向，世界系）
                    local perp_x =  math.cos(expected_bearing_rad)
                    local perp_z = -math.sin(expected_bearing_rad)
                    local pa = C.NAV_POS_ALPHA or 0.05
                    self.x = self.x + pa * lateral_err * perp_x
                    self.z = self.z + pa * lateral_err * perp_z
                end
            end
        end
    end

    self._init = true
    return self
end

-- 速度传感器偏置标定（arm前静止时调用，采样N次取均值）
function IMU:calibrate(samples)
    samples = samples or 10
    local sx, sz = 0, 0
    local cx, cz = 0, 0
    for _ = 1, samples do
        if self.vel_x_p then
            local ok, v = pcall(function() return self.vel_x_p.getVelocity() end)
            if ok and type(v) == "number" then sx = sx + v; cx = cx + 1 end
        end
        if self.vel_z_p then
            local ok, v = pcall(function() return self.vel_z_p.getVelocity() end)
            if ok and type(v) == "number" then sz = sz + v; cz = cz + 1 end
        end
        os.sleep(0.05)
    end
    if cx > 0 then self._vx_bias = sx / cx end
    if cz > 0 then self._vz_bias = sz / cz end
    return self._vx_bias, self._vz_bias
end

-- 单独调用，放在慢速循环（1~2Hz），避免阻塞控制环
function IMU:readGPS()
    -- GPS 已禁用
end

function IMU:status()
    local function yn(p) return p and "OK" or "--" end
    return string.format("gim:%s alt:%s vx:%s vz:%s nav:%s",
        yn(self.gim_p), yn(self.alt_p), yn(self.vel_x_p), yn(self.vel_z_p), yn(self.nav_p))
end

return IMU
