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
    self.rpm = {0, 0, 0, 0}
    return self
end

-- ── internal: apply 4 RPM values ─────────────────────────────
local function applyRPMs(self, r1, r2, r3, r4)
    self.rpm = {r1, r2, r3, r4}
    if use_slaves then
        -- send to each slave computer
        local rpms = {r1, r2, r3, r4}
        for i = 1, 4 do
            if slave_ids[i] then
                rednet.send(slave_ids[i],
                    { idx = i, rpm = math.floor(rpms[i] + 0.5) },
                    C.MOTOR_PROTOCOL)
            end
        end
    else
        local function set(motor, rpm)
            if not motor then return end
            rpm = math.max(C.RPM_MIN, math.min(C.RPM_MAX, rpm))
            motor.p.setTargetSpeed(math.floor(rpm + 0.5))
        end
        set(self.motors[1], r1)
        set(self.motors[2], r2)
        set(self.motors[3], r3)
        set(self.motors[4], r4)
    end
end

-- ── mix and output ────────────────────────────────────────────
function Mixer:mix(throttle, pitch_out, roll_out, yaw_out)
    -- attitude authority: max RPM delta for full deflection
    local dp = pitch_out * 20   -- max ±20 RPM for pitch
    local dr = roll_out  * 20   -- max ±20 RPM for roll
    local dy = yaw_out   * 12   -- max ±12 RPM for yaw

    -- negate dp/dr: sensor positive = nose-up/right-tilt,
    -- correction needs opposite motor response
    local r1 = throttle - dp - dr - dy   -- M1 FL CCW
    local r2 = throttle - dp + dr + dy   -- M2 FR CW
    local r3 = throttle + dp + dr - dy   -- M3 BR CCW
    local r4 = throttle + dp - dr + dy   -- M4 BL CW

    local max_r = math.max(r1, r2, r3, r4)
    local min_r = math.min(r1, r2, r3, r4)
    if max_r > C.RPM_MAX then
        local e = max_r - C.RPM_MAX
        r1=r1-e; r2=r2-e; r3=r3-e; r4=r4-e
    end
    if min_r < C.RPM_MIN then
        local e = C.RPM_MIN - min_r
        r1=r1+e; r2=r2+e; r3=r3+e; r4=r4+e
    end

    applyRPMs(self, r1, r2, r3, r4)
end

function Mixer:allStop()
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
