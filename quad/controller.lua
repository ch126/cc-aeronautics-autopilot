-- =============================================================
--  quad/controller.lua
--  Cascade PID: rate loop(50Hz) -> attitude loop(50Hz) -> alt/pos loop(10Hz)
--  Uses autopilot/pid.lua  API:
--    PID.new({kp,ki,kd,integral_max,output_max})
--    pid:compute(setpoint, measured, dt)  -> output
--    pid:reset()
-- =============================================================

local C       = dofile("/quad/config.lua")
local PID_lib = dofile("/autopilot/pid.lua")
local PID     = PID_lib.PID

-- convert config table (uses imax/omax) to pid.lua format
local function make_pid(cfg)
    return PID.new({
        kp           = cfg.kp   or 1.0,
        ki           = cfg.ki   or 0.0,
        kd           = cfg.kd   or 0.0,
        integral_max = cfg.imax or cfg.integral_max or math.huge,
        output_max   = cfg.omax or cfg.output_max   or math.huge,
    })
end

local Ctrl = {}
Ctrl.__index = Ctrl

function Ctrl.new()
    local self = setmetatable({}, Ctrl)

    -- rate loop PIDs  (input: rate error deg/s -> output: normalised -1..1)
    self.rate_p = make_pid(C.PID_RATE_PITCH)
    self.rate_q = make_pid(C.PID_RATE_ROLL)
    self.rate_r = make_pid(C.PID_RATE_YAW)

    -- attitude loop PIDs  (input: angle error deg -> output: target rate deg/s)
    self.att_p  = make_pid(C.PID_ATT_PITCH)
    self.att_q  = make_pid(C.PID_ATT_ROLL)
    self.att_r  = make_pid(C.PID_ATT_YAW)

    -- targets
    self.target_alt   = 0
    self.target_yaw   = 0
    self.target_pitch = 0
    self.target_roll  = 0
    self.target_x     = nil
    self.target_z     = nil

    self.throttle_out = 0
    self.armed        = false
    self._alt_i       = 0

    return self
end

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end

-- shortest angle difference handling 360/0 wrap
local function angDiff(target, current)
    local d = (target - current) % 360
    if d > 180 then d = d - 360 end
    return d
end

-- outer loop: altitude + position  (call at NAV_HZ ~10Hz)
function Ctrl:updateOuter(imu_state, dt)
    if not self.armed then return end

    local alt   = imu_state.altitude   or 0
    local climb = imu_state.climb_rate or 0
    local pitch = imu_state.pitch      or 0
    local roll  = imu_state.roll       or 0

    -- ── 高度级联：误差 → 目标爬升率 → 油门增量 ──────────────
    local alt_err      = self.target_alt - alt
    local target_climb = clamp(alt_err * C.ALT_POS_GAIN, -C.ALT_MAX_CLIMB, C.ALT_MAX_CLIMB)
    local climb_err    = target_climb - climb
    local thr_delta    = clamp(climb_err * C.ALT_VEL_GAIN, -C.ALT_MAX_DELTA, C.ALT_MAX_DELTA)

    -- 慢速积分器：消除悬停偏置
    self._alt_i = self._alt_i + alt_err * C.ALT_I_GAIN * dt
    self._alt_i = clamp(self._alt_i, -C.ALT_I_MAX, C.ALT_I_MAX)

    -- ── 倾斜补偿（真实飞控核心）───────────────────────────────
    -- 机体倾斜时垂直推力 = throttle * cos(pitch) * cos(roll)
    -- 补偿：将油门除以倾斜因子，维持恒定升力
    local cp = math.cos(math.rad(pitch))
    local cr = math.cos(math.rad(roll))
    local tilt_factor = math.max(cp * cr, 0.75)  -- 限制最大补偿25%，减小震荡正反馈
    local base_thr = clamp(C.RPM_HOVER + thr_delta + self._alt_i, C.RPM_MIN, C.RPM_MAX)
    self.throttle_out = clamp(base_thr / tilt_factor, C.RPM_MIN, C.RPM_MAX)

    -- ── 水平速度阻尼 ──────────────────────────────────────────
    local vx = imu_state.vx or 0
    local vz = imu_state.vz or 0
    local cy = math.cos(math.rad(imu_state.yaw or 0))
    local sy = math.sin(math.rad(imu_state.yaw or 0))

    local target_vx, target_vz = 0, 0

    if self.target_x and imu_state.x then
        local ex = self.target_x - imu_state.x
        local ez = self.target_z - imu_state.z
        local dist = math.sqrt(ex*ex + ez*ez)
        if dist > C.POS_DEADBAND then
            target_vx = clamp(ex * C.POS_GAIN, -C.POS_MAX_VEL, C.POS_MAX_VEL)
            target_vz = clamp(ez * C.POS_GAIN, -C.POS_MAX_VEL, C.POS_MAX_VEL)
        end
    end

    -- 速度误差（世界系）→ 机体系 → 期望倾斜角
    local dvx   = target_vx - vx
    local dvz   = target_vz - vz
    local dvx_b =  cy * dvx + sy * dvz
    local dvz_b = -sy * dvx + cy * dvz

    -- 速度死区：绝对速度很小时不产生倾斜指令，避免噪声引发振荡
    local VEL_DB = 0.12  -- m/s
    if math.abs(vx) < VEL_DB and math.abs(vz) < VEL_DB and target_vx == 0 and target_vz == 0 then
        dvx_b, dvz_b = 0, 0
    end

    local raw_pitch = clamp(-dvx_b * C.VEL_GAIN, -C.ATT_MAX, C.ATT_MAX)
    local raw_roll  = clamp(-dvz_b * C.VEL_GAIN, -C.ATT_MAX, C.ATT_MAX)

    -- ── 设定值平滑（一阶低通，防止阶跃输入）─────────────────
    -- 类似真实FC的 tpa / setpoint smoothing
    local SP_ALPHA = C.SP_SMOOTH or 0.3  -- 0=完全平滑, 1=无平滑
    self.target_pitch = self.target_pitch + SP_ALPHA * (raw_pitch - self.target_pitch)
    self.target_roll  = self.target_roll  + SP_ALPHA * (raw_roll  - self.target_roll)
end

-- inner loop: attitude + rate  (call at CTRL_HZ ~50Hz)
-- returns pitch_out, roll_out, yaw_out  (-1..1)
function Ctrl:updateInner(imu_state, dt)
    if not self.armed then return 0, 0, 0 end

    -- attitude loop: angle error -> target rate
    local rate_p_tgt = clamp(
        self.att_p:compute(self.target_pitch, imu_state.pitch or 0, dt),
        -C.RATE_MAX, C.RATE_MAX)
    local rate_q_tgt = clamp(
        self.att_q:compute(self.target_roll,  imu_state.roll  or 0, dt),
        -C.RATE_MAX, C.RATE_MAX)
    -- yaw: use angle difference to handle 360/0 wrap
    local yaw_err    = angDiff(self.target_yaw, imu_state.yaw or 0)
    local rate_r_tgt = clamp(
        self.att_r:compute(yaw_err, 0, dt),
        -C.RATE_MAX, C.RATE_MAX)

    -- rate loop: rate error -> output
    local pitch_out = clamp(
        self.rate_p:compute(rate_p_tgt, imu_state.rate_p or 0, dt), -1, 1)
    local roll_out  = clamp(
        self.rate_q:compute(rate_q_tgt, imu_state.rate_q or 0, dt), -1, 1)
    local yaw_out   = clamp(
        self.rate_r:compute(rate_r_tgt, imu_state.rate_r or 0, dt), -1, 1)

    return pitch_out, roll_out, yaw_out
end

function Ctrl:arm(alt, yaw)
    self.armed        = true
    self.target_alt   = alt or 0
    self.target_yaw   = yaw or 0
    self.target_pitch = 0
    self.target_roll  = 0
    self.throttle_out = C.RPM_HOVER
    self._alt_i       = 0
    -- reset all integrators
    for _, p in ipairs({
        self.rate_p, self.rate_q, self.rate_r,
        self.att_p,  self.att_q,  self.att_r,
    }) do p:reset() end
end

function Ctrl:disarm()
    self.armed        = false
    self.throttle_out = 0
    self.target_x     = nil
    self.target_z     = nil
end

function Ctrl:hover(imu_state)
    self.target_pitch = 0
    self.target_roll  = 0
    if imu_state then
        self.target_alt = imu_state.altitude or self.target_alt
        self.target_yaw = imu_state.yaw      or self.target_yaw
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
