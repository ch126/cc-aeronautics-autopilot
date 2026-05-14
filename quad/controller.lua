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

    -- altitude loop PID  (input: alt error m -> output: throttle delta RPM)
    self.alt    = make_pid(C.PID_ALT)

    -- position loop PIDs  (input: pos error m -> output: target pitch/roll deg)
    self.pos_x  = make_pid(C.PID_POS_X)
    self.pos_z  = make_pid(C.PID_POS_Z)

    -- targets
    self.target_alt   = 0
    self.target_yaw   = 0
    self.target_pitch = 0
    self.target_roll  = 0
    self.target_x     = nil
    self.target_z     = nil

    self.throttle_out = 0
    self.armed        = false

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

    -- altitude loop: PID on height error + climb_rate damping
    local alt      = imu_state.altitude   or 0
    local climb    = imu_state.climb_rate or 0
    local alt_err  = self.target_alt - alt

    -- integrator freeze when climbing away from target (anti-windup)
    local freeze_i = (alt_err > 0 and climb > 1.0) or (alt_err < 0 and climb < -1.0)
    if freeze_i then self.alt:reset() end

    local thr_delta = self.alt:compute(self.target_alt, alt, dt)
    -- subtract climb_rate damping to brake before reaching target
    thr_delta = thr_delta - climb * 8.0

    self.throttle_out = clamp(C.RPM_HOVER + thr_delta, C.RPM_MIN, C.RPM_MAX)

    -- position loop (needs GPS)
    if self.target_x and imu_state.x then
        local ex = self.target_x - imu_state.x
        local ez = self.target_z - (imu_state.z or 0)
        -- rotate error into body frame
        local cy = math.cos(math.rad(imu_state.yaw or 0))
        local sy = math.sin(math.rad(imu_state.yaw or 0))
        local ex_b =  cy * ex + sy * ez
        local ez_b = -sy * ex + cy * ez
        self.target_pitch = clamp(
            self.pos_x:compute(0, -ex_b, dt), -C.ATT_MAX, C.ATT_MAX)
        self.target_roll  = clamp(
            self.pos_z:compute(0, -ez_b, dt), -C.ATT_MAX, C.ATT_MAX)
    end
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
    -- reset all integrators
    for _, p in ipairs({
        self.rate_p, self.rate_q, self.rate_r,
        self.att_p,  self.att_q,  self.att_r,
        self.alt,    self.pos_x,  self.pos_z,
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
