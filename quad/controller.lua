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
        deriv_max    = cfg.dmax or cfg.deriv_max    or math.huge,
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

    -- ── 水平位置保持 ──────────────────────────────────────────
    local vx = imu_state.vx or 0
    local vz = imu_state.vz or 0
    local cy = math.cos(math.rad(imu_state.yaw or 0))
    local sy = math.sin(math.rad(imu_state.yaw or 0))

    local raw_pitch, raw_roll = 0, 0

    -- ── Nav Follow 模式：跟着导航台方向飞 ─────────────────────
    -- nav_follow_speed 不为 nil 时，忽略 target_x/z，
    -- 按导航台指向以固定速度飞行（yaw=0 为参考方向，rel 为机头相对偏角）
    if self.nav_follow_speed and imu_state.nav_rel ~= nil then
        local rel_rad     = math.rad(imu_state.nav_rel)
        local yaw_rad     = math.rad(imu_state.yaw or 0)
        local world_bear  = yaw_rad + rel_rad
        local spd         = self.nav_follow_speed
        local target_vx   = spd * math.sin(world_bear)
        local target_vz   = spd * math.cos(world_bear)
        local dvx   = target_vx - vx
        local dvz   = target_vz - vz
        local dvx_b =  cy * dvx + sy * dvz
        local dvz_b = -sy * dvx + cy * dvz
        raw_pitch = clamp(-dvx_b * C.VEL_GAIN, -C.ATT_MAX, C.ATT_MAX)
        raw_roll  = clamp(-dvz_b * C.VEL_GAIN, -C.ATT_MAX, C.ATT_MAX)
        self.dbg = {
            ex=0, ez=0, dist=0,
            tvx=target_vx, tvz=target_vz,
            dvx_b=dvx_b, dvz_b=dvz_b,
            rp=raw_pitch, rr=raw_roll,
            ix=0, iz=0,
            nav_rel=imu_state.nav_rel,
        }
    elseif self.target_x and imu_state.x then
        local ex = self.target_x - imu_state.x
        local ez = self.target_z - imu_state.z
        local dist = math.sqrt(ex*ex + ez*ez)

        -- ── 到达检测 ────────────────────────────────────────────
        -- dist < ARRIVAL_RADIUS 时：冻结目标为当前位置，切换到悬停定点
        -- 积分器保留（不清零），避免立刻再漂移
        local arrival_r = C.ARRIVAL_RADIUS or 2.0
        if self._goto_active and dist < arrival_r then
            self._goto_active = false
            self.arrived      = true   -- 供 main.lua 检测
            -- 锁定当前位置（消除追逐）
            self.target_x = imu_state.x
            self.target_z = imu_state.z
            ex, ez = 0, 0
            dist   = 0
        end

        -- 位置积分器：消除稳态偏差（风/推力偏差）
        -- 只在 GPS 有效且误差较小时积分，避免积分饱和
        if imu_state.gps_ok and dist < 5.0 then
            self._pos_ix = (self._pos_ix or 0) + ex * C.POS_I_GAIN * dt
            self._pos_iz = (self._pos_iz or 0) + ez * C.POS_I_GAIN * dt
            self._pos_ix = clamp(self._pos_ix, -C.POS_I_MAX, C.POS_I_MAX)
            self._pos_iz = clamp(self._pos_iz, -C.POS_I_MAX, C.POS_I_MAX)
        end

        -- 位置 → 目标速度（P）+ 积分（I）
        local target_vx = clamp(ex * C.POS_GAIN + (self._pos_ix or 0), -C.POS_MAX_VEL, C.POS_MAX_VEL)
        local target_vz = clamp(ez * C.POS_GAIN + (self._pos_iz or 0), -C.POS_MAX_VEL, C.POS_MAX_VEL)

        -- 速度误差（世界系→机体系）
        local dvx   = target_vx - vx
        local dvz   = target_vz - vz
        local dvx_b =  cy * dvx + sy * dvz
        local dvz_b = -sy * dvx + cy * dvz

        raw_pitch = clamp(-dvx_b * C.VEL_GAIN, -C.ATT_MAX, C.ATT_MAX)
        raw_roll  = clamp(-dvz_b * C.VEL_GAIN, -C.ATT_MAX, C.ATT_MAX)

        -- 诊断
        self.dbg = {
            ex=ex, ez=ez, dist=dist,
            tvx=target_vx, tvz=target_vz,
            dvx_b=dvx_b, dvz_b=dvz_b,
            rp=raw_pitch, rr=raw_roll,
            ix=self._pos_ix or 0, iz=self._pos_iz or 0,
        }
    else
        -- 无位置目标：纯速度阻尼
        self._pos_ix = 0
        self._pos_iz = 0
        local dvx_b =  cy * (0 - vx) + sy * (0 - vz)
        local dvz_b = -sy * (0 - vx) + cy * (0 - vz)
        local VEL_DB = 0.25
        if math.abs(vx) < VEL_DB and math.abs(vz) < VEL_DB then
            dvx_b, dvz_b = 0, 0
        end
        raw_pitch = clamp(-dvx_b * C.VEL_GAIN, -C.ATT_MAX, C.ATT_MAX)
        raw_roll  = clamp(-dvz_b * C.VEL_GAIN, -C.ATT_MAX, C.ATT_MAX)
        self.dbg = nil
    end

    local has_pos_target = (self.target_x ~= nil)

    -- ── 设定值平滑（一阶低通）────────────────────────────────
    local SP_ALPHA = has_pos_target and 0.3 or (C.SP_SMOOTH or 0.08)
    self.target_pitch = self.target_pitch + SP_ALPHA * (raw_pitch - self.target_pitch)
    self.target_roll  = self.target_roll  + SP_ALPHA * (raw_roll  - self.target_roll)

    -- 无位置目标时缓慢衰减防漂移
    if not has_pos_target then
        self.target_pitch = self.target_pitch * 0.95
        self.target_roll  = self.target_roll  * 0.95
    end
end

-- inner loop: attitude  (call at CTRL_HZ ~20Hz)
-- returns pitch_out, roll_out, yaw_out  (-1..1)
function Ctrl:updateInner(imu_state, dt)
    if not self.armed then return 0, 0, 0 end

    -- 单环姿态控制：角度误差 → 直接输出（跳过 rate loop）
    -- 有限差分估算的角速度太噪，rate loop 在20Hz下会放大震荡
    local pitch_out = clamp(
        self.att_p:compute(self.target_pitch, imu_state.pitch or 0, dt) / C.RATE_MAX,
        -1, 1)
    local roll_out  = clamp(
        self.att_q:compute(self.target_roll,  imu_state.roll  or 0, dt) / C.RATE_MAX,
        -1, 1)
    local yaw_err   = angDiff(self.target_yaw, imu_state.yaw or 0)
    local yaw_out   = clamp(
        self.att_r:compute(yaw_err, 0, dt) / C.RATE_MAX,
        -1, 1)

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
    self._pos_ix      = 0
    self._pos_iz      = 0
    self._goto_active = false
    self.arrived      = false
    self.nav_follow_speed = nil
    -- reset all integrators
    for _, p in ipairs({
        self.att_p,  self.att_q,  self.att_r,
    }) do p:reset() end
end

function Ctrl:disarm()
    self.armed        = false
    self.throttle_out = 0
    self.target_x     = nil
    self.target_z     = nil
    self._pos_ix      = 0
    self._pos_iz      = 0
    self.nav_follow_speed = nil
end

function Ctrl:hover(imu_state)
    self.target_pitch     = 0
    self.target_roll      = 0
    self.nav_follow_speed = nil   -- 停止 nav follow
    if imu_state then
        self.target_alt = imu_state.altitude or self.target_alt
        self.target_yaw = imu_state.yaw      or self.target_yaw
    end
    -- 锁定当前位置为目标（位置保持已激活时）
    -- 切换目标时重置积分器防止积分饱和
    if self.target_x ~= nil and imu_state and imu_state.x ~= nil then
        self.target_x = imu_state.x
        self.target_z = imu_state.z or self.target_z
        self._pos_ix  = 0
        self._pos_iz  = 0
    else
        self.target_x = nil
        self.target_z = nil
        self._pos_ix  = 0
        self._pos_iz  = 0
    end
end

return Ctrl
