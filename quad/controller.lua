-- =============================================================
--  quad/controller.lua
--  级联PID飞控：速率环(50Hz) → 姿态环(50Hz) → 位置/高度环(10Hz)
-- =============================================================

local C   = dofile("/quad/config.lua")
local PID = dofile("/autopilot/pid.lua")

local Ctrl = {}
Ctrl.__index = Ctrl

function Ctrl.new()
    local self = setmetatable({}, Ctrl)

    -- 速率环 PID（输入: 角速率误差 deg/s，输出: 转速差量标幺值 -1..1）
    self.rate_p = PID.new(C.PID_RATE_PITCH.kp, C.PID_RATE_PITCH.ki, C.PID_RATE_PITCH.kd, C.PID_RATE_PITCH.ilim)
    self.rate_q = PID.new(C.PID_RATE_ROLL.kp,  C.PID_RATE_ROLL.ki,  C.PID_RATE_ROLL.kd,  C.PID_RATE_ROLL.ilim)
    self.rate_r = PID.new(C.PID_RATE_YAW.kp,   C.PID_RATE_YAW.ki,  C.PID_RATE_YAW.kd,   C.PID_RATE_YAW.ilim)

    -- 姿态环 PID（输入: 角度误差 deg，输出: 目标角速率 deg/s）
    self.att_p  = PID.new(C.PID_ATT_PITCH.kp, C.PID_ATT_PITCH.ki, C.PID_ATT_PITCH.kd, C.PID_ATT_PITCH.ilim)
    self.att_q  = PID.new(C.PID_ATT_ROLL.kp,  C.PID_ATT_ROLL.ki,  C.PID_ATT_ROLL.kd,  C.PID_ATT_ROLL.ilim)
    self.att_r  = PID.new(C.PID_ATT_YAW.kp,   C.PID_ATT_YAW.ki,  C.PID_ATT_YAW.kd,   C.PID_ATT_YAW.ilim)

    -- 高度环 PID（输入: 高度误差 m，输出: 油门基准 RPM）
    self.alt    = PID.new(C.PID_ALT.kp, C.PID_ALT.ki, C.PID_ALT.kd, C.PID_ALT.ilim)

    -- 水平位置环 PID（输入: 位置误差 m，输出: 目标俯仰/横滚角 deg）
    self.pos_x  = PID.new(C.PID_POS_X.kp, C.PID_POS_X.ki, C.PID_POS_X.kd, C.PID_POS_X.ilim)
    self.pos_z  = PID.new(C.PID_POS_Z.kp, C.PID_POS_Z.ki, C.PID_POS_Z.kd, C.PID_POS_Z.ilim)

    -- 目标状态
    self.target_alt   = 0      -- 目标高度 (m)
    self.target_yaw   = 0      -- 目标偏航 (deg)
    self.target_pitch = 0      -- 目标俯仰角 (deg)，由位置环写入
    self.target_roll  = 0      -- 目标横滚角 (deg)，由位置环写入
    self.target_x     = nil    -- 目标位置 X (nil=不控制)
    self.target_z     = nil    -- 目标位置 Z
    self.armed        = false

    return self
end

-- 角度差（处理 ±180 折叠）
local function angDiff(a, b)
    local d = (a - b) % 360
    if d > 180 then d = d - 360 end
    return d
end

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end

-- 位置/高度外环（低频调用，10Hz）
-- imu_state: IMU:read() 返回值
-- 输出写入 self.target_pitch, target_roll, 并更新 alt PID 输出
function Ctrl:updateOuter(imu_state, dt)
    if not self.armed then return end

    -- 高度环
    local alt_err = self.target_alt - imu_state.alt
    self.throttle_out = C.RPM_HOVER + self.alt:update(alt_err, dt)
    self.throttle_out = clamp(self.throttle_out, C.RPM_MIN, C.RPM_MAX)

    -- 水平位置环（需要 GPS）
    if self.target_x and imu_state.x then
        local ex = self.target_x - imu_state.x
        local ez = self.target_z - imu_state.z
        -- 位置误差投影到机体系（考虑偏航）
        local cos_y = math.cos(math.rad(imu_state.yaw))
        local sin_y = math.sin(math.rad(imu_state.yaw))
        local ex_b =  cos_y * ex + sin_y * ez
        local ez_b = -sin_y * ex + cos_y * ez
        -- 位置→目标角度
        self.target_pitch = clamp(self.pos_x:update(ex_b, dt), -C.ATT_MAX, C.ATT_MAX)
        self.target_roll  = clamp(self.pos_z:update(ez_b, dt), -C.ATT_MAX, C.ATT_MAX)
    end
end

-- 姿态+速率内环（高频调用，50Hz）
-- 返回 pitch_out, roll_out, yaw_out（-1..1 标幺值，传给 mixer）
function Ctrl:updateInner(imu_state, dt)
    if not self.armed then
        return 0, 0, 0
    end

    -- 姿态环：角度误差 → 目标角速率
    local ep = angDiff(self.target_pitch, imu_state.pitch)
    local er = angDiff(self.target_roll,  imu_state.roll)
    local ey = angDiff(self.target_yaw,   imu_state.yaw)

    local rate_p_tgt = clamp(self.att_p:update(ep, dt), -C.RATE_MAX, C.RATE_MAX)
    local rate_q_tgt = clamp(self.att_q:update(er, dt), -C.RATE_MAX, C.RATE_MAX)
    local rate_r_tgt = clamp(self.att_r:update(ey, dt), -C.RATE_MAX, C.RATE_MAX)

    -- 速率环：角速率误差 → 转速标幺值
    local ep2 = rate_p_tgt - imu_state.rate_p
    local eq2 = rate_q_tgt - imu_state.rate_q
    local er2 = rate_r_tgt - imu_state.rate_r

    local pitch_out = clamp(self.rate_p:update(ep2, dt), -1, 1)
    local roll_out  = clamp(self.rate_q:update(eq2, dt), -1, 1)
    local yaw_out   = clamp(self.rate_r:update(er2, dt), -1, 1)

    return pitch_out, roll_out, yaw_out
end

-- 解锁
function Ctrl:arm(alt, yaw)
    self.armed = true
    self.target_alt = alt or 0
    self.target_yaw = yaw or 0
    self.throttle_out = C.RPM_HOVER
    -- 重置所有积分
    for _, pid in ipairs({
        self.rate_p, self.rate_q, self.rate_r,
        self.att_p, self.att_q, self.att_r,
        self.alt, self.pos_x, self.pos_z
    }) do
        pid:reset()
    end
end

-- 加锁
function Ctrl:disarm()
    self.armed = false
    self.throttle_out = 0
    self.target_x = nil
    self.target_z = nil
end

-- 悬停（保持当前位置）
function Ctrl:hover(imu_state)
    self.target_pitch = 0
    self.target_roll  = 0
    if imu_state then
        self.target_alt = imu_state.alt
        self.target_yaw = imu_state.yaw
        if imu_state.x then
            self.target_x = imu_state.x
            self.target_z = imu_state.z
        else
            self.target_x = nil
            self.target_z = nil
        end
    end
end

return Ctrl
