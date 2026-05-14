-- =============================================================
--  quad/mixer.lua
--  X-type quadrotor mixing matrix + speed controller output
--
--  Supports two modes:
--    1. SLAVE MODE  (C.SLAVE_FL/FR/BR/BL set in config.lua)
--       Master sends RPM via rednet to 4 slave computers.
--       Each slave runs /quad/slave and controls one motor.
--
--    2. DIRECT MODE (slaves = nil, default)
--       Master directly wraps Create_RotationSpeedController
--       peripherals via wired modem.
--
--  Motor layout (top view):
--         FRONT
--    M1(CCW)   M2(CW)
--      \         /
--       +---------+
--      /         \
--    M4(CW)    M3(CCW)
--         REAR
-- =============================================================

local C = dofile("/quad/config.lua")

local Mixer = {}
Mixer.__index = Mixer

-- ── detect mode ───────────────────────────────────────────────
local slave_ids  = { C.SLAVE_FL, C.SLAVE_FR, C.SLAVE_BR, C.SLAVE_BL }
local use_slaves = C.SLAVE_FL or C.SLAVE_FR or C.SLAVE_BR or C.SLAVE_BL

-- ── direct mode: find local peripherals ──────────────────────
local function findMotors()
    local motors = {}
    local names  = { C.MOTOR_FL, C.MOTOR_FR, C.MOTOR_BR, C.MOTOR_BL }
    local labels = { "FL", "FR", "BR", "BL" }
    local found_all = true
    for i, name in ipairs(names) do
        if type(name) == "string" then
            local p = peripheral.wrap(name)
            if p then
                motors[i] = { p=p, name=name, label=labels[i] }
            else
                print("WARN: motor " .. labels[i] .. " not found: " .. name)
                found_all = false
            end
        else
            found_all = false
        end
    end
    if not found_all then
        local auto_list = { peripheral.find(C.MOTOR_TYPE) }
        local auto_idx  = 1
        for i = 1, 4 do
            if not motors[i] then
                if auto_list[auto_idx] then
                    motors[i] = {
                        p     = auto_list[auto_idx],
                        name  = "auto_" .. auto_idx,
                        label = labels[i]
                    }
                    auto_idx = auto_idx + 1
                end
            end
        end
    end
    return motors
end

-- ── slave mode: open rednet ───────────────────────────────────
local rednet_opened = false
local function ensureRednet()
    if rednet_opened then return end
    peripheral.find("modem", rednet.open)
    rednet_opened = true
end

-- ── constructor ───────────────────────────────────────────────
function Mixer.new()
    local self = setmetatable({}, Mixer)
    if use_slaves then
        ensureRednet()
        self.motors = nil
        print("Mixer: slave mode  FL=" .. tostring(C.SLAVE_FL)
            .. " FR=" .. tostring(C.SLAVE_FR)
            .. " BR=" .. tostring(C.SLAVE_BR)
            .. " BL=" .. tostring(C.SLAVE_BL))
    else
        self.motors = findMotors()
        local cnt = 0
        for i = 1, 4 do if self.motors[i] then cnt = cnt + 1 end end
        print("Mixer: direct mode  " .. cnt .. "/4 motors found")
    end
    self.rpm      = {0, 0, 0, 0}
    self.rpm_filt = {0, 0, 0, 0}  -- 低通滤波后的实际发送值（模拟电机惯性）
    return self
end

-- ── internal: apply 4 RPM values ─────────────────────────────
-- MOTOR_ALPHA: 电机一阶低通滤波系数
-- 真实螺旋桨有惯性，RPM不能瞬间到达目标值
-- alpha越小=电机响应越慢（越重的桨用越小的值）
-- 20Hz下 alpha=0.4 约等于时间常数 ~75ms
local MOTOR_ALPHA = 0.4

local function applyRPMs(self, r1, r2, r3, r4)
    self.rpm = {r1, r2, r3, r4}
    -- 一阶低通：模拟电机加速惯性
    local f = self.rpm_filt
    f[1] = f[1] + MOTOR_ALPHA * (r1 - f[1])
    f[2] = f[2] + MOTOR_ALPHA * (r2 - f[2])
    f[3] = f[3] + MOTOR_ALPHA * (r3 - f[3])
    f[4] = f[4] + MOTOR_ALPHA * (r4 - f[4])
    if use_slaves then
        -- send filtered RPM to each slave computer
        for i = 1, 4 do
            if slave_ids[i] then
                rednet.send(slave_ids[i],
                    { idx = i, rpm = math.floor(f[i] + 0.5) },
                    C.MOTOR_PROTOCOL)
            end
        end
    else
        local function set(motor, rpm)
            if not motor then return end
            rpm = math.max(C.RPM_MIN, math.min(C.RPM_MAX, rpm))
            motor.p.setTargetSpeed(math.floor(rpm + 0.5))
        end
        set(self.motors[1], f[1])
        set(self.motors[2], f[2])
        set(self.motors[3], f[3])
        set(self.motors[4], f[4])
    end
end

-- ── mix and output ────────────────────────────────────────────
function Mixer:mix(throttle, pitch_out, roll_out, yaw_out)
    local dp = pitch_out * 20
    local dr = roll_out  * 20
    local dy = yaw_out   * 12

    local r1 = throttle - dp - dr - dy   -- M1 FL CCW
    local r2 = throttle - dp + dr + dy   -- M2 FR CW
    local r3 = throttle + dp + dr - dy   -- M3 BR CCW
    local r4 = throttle + dp - dr + dy   -- M4 BL CW

    -- ── 优先级饱和（真实FC desaturation）────────────────────
    -- 超上限：先削减yaw，再削减pitch/roll，最后才动throttle
    local function sat_high()
        local over = math.max(r1,r2,r3,r4) - C.RPM_MAX
        if over <= 0 then return end
        -- 1. 削减yaw authority
        local dy_cut = math.min(over/2, math.abs(dy))
        local yaw_scale = (math.abs(dy) > 0.01) and (1 - dy_cut/math.abs(dy)) or 1
        dy = dy * yaw_scale
        r1 = throttle - dp - dr - dy
        r2 = throttle - dp + dr + dy
        r3 = throttle + dp + dr - dy
        r4 = throttle + dp - dr + dy
        -- 2. 仍超限则整体下移（保持姿态，牺牲throttle）
        over = math.max(r1,r2,r3,r4) - C.RPM_MAX
        if over > 0 then
            r1=r1-over; r2=r2-over; r3=r3-over; r4=r4-over
        end
    end

    local function sat_low()
        local under = C.RPM_MIN - math.min(r1,r2,r3,r4)
        if under <= 0 then return end
        local dy_cut = math.min(under/2, math.abs(dy))
        local yaw_scale = (math.abs(dy) > 0.01) and (1 - dy_cut/math.abs(dy)) or 1
        dy = dy * yaw_scale
        r1 = throttle - dp - dr - dy
        r2 = throttle - dp + dr + dy
        r3 = throttle + dp + dr - dy
        r4 = throttle + dp - dr + dy
        under = C.RPM_MIN - math.min(r1,r2,r3,r4)
        if under > 0 then
            r1=r1+under; r2=r2+under; r3=r3+under; r4=r4+under
        end
    end

    sat_high()
    sat_low()

    applyRPMs(self, r1, r2, r3, r4)
end

function Mixer:allStop()
    self.rpm_filt = {0, 0, 0, 0}
    applyRPMs(self, 0, 0, 0, 0)
end

function Mixer:status()
    local s = ""
    local labels = {"FL","FR","BR","BL"}
    for i = 1, 4 do
        local ok
        if use_slaves then
            ok = slave_ids[i] and ("S"..tostring(slave_ids[i])) or "--"
        else
            ok = (self.motors and self.motors[i]) and "OK" or "--"
        end
        s = s .. string.format("%s:%s(%3d) ", labels[i], ok, self.rpm[i])
    end
    return s
end

return Mixer
