-- =============================================================
--  quad/mixer.lua
--  X-type quadrotor mixing matrix + speed controller output
--
--  Motor layout (top view):
--
--         FRONT
--    M1(CCW)   M2(CW)
--      \         /
--       +---------+
--      /         \
--    M4(CW)    M3(CCW)
--         REAR
--
--  Mixing:
--    M1 = throttle + pitch + roll - yaw   (FL, CCW)
--    M2 = throttle + pitch - roll + yaw   (FR, CW)
--    M3 = throttle - pitch - roll - yaw   (BR, CCW)
--    M4 = throttle - pitch + roll + yaw   (BL, CW)
--
--  Create speed controller: setTargetSpeed(rpm)
-- =============================================================

local C = dofile("/quad/config.lua")

local Mixer = {}
Mixer.__index = Mixer

-- find speed controllers, assign FL/FR/BR/BL in order
local function findMotors()
    local motors = {}
    local names = { C.MOTOR_FL, C.MOTOR_FR, C.MOTOR_BR, C.MOTOR_BL }
    local labels = { "FL", "FR", "BR", "BL" }

    local found_all = true
    for i, name in ipairs(names) do
        if name then
            local p = peripheral.wrap(name)
            if p then
                motors[i] = { p=p, name=name, label=labels[i] }
            else
                print("WARN: motor " .. labels[i] .. " '" .. name .. "' not found")
                found_all = false
            end
        else
            found_all = false
        end
    end

    -- auto-scan missing motors
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
                else
                    motors[i] = nil
                end
            end
        end
    end
    return motors
end

function Mixer.new()
    local self = setmetatable({}, Mixer)
    self.motors = findMotors()
    self.rpm = {0, 0, 0, 0}
    return self
end

local function setMotorRPM(motor, rpm)
    if not motor then return end
    rpm = math.max(C.RPM_MIN, math.min(C.RPM_MAX, rpm))
    local ok = pcall(motor.p.setTargetSpeed, math.floor(rpm + 0.5))
    if not ok then
        pcall(motor.p.setSpeed, math.floor(rpm + 0.5))
    end
end

-- throttle: 0..RPM_MAX  (hover ~RPM_HOVER)
-- pitch_out, roll_out, yaw_out: -1..1
function Mixer:mix(throttle, pitch_out, roll_out, yaw_out)
    local half_range = (C.RPM_MAX - C.RPM_MIN) * 0.5
    local dp = pitch_out * half_range * 0.5
    local dr = roll_out  * half_range * 0.5
    local dy = yaw_out   * half_range * 0.3

    local r1 = throttle + dp + dr - dy   -- M1 FL CCW
    local r2 = throttle + dp - dr + dy   -- M2 FR CW
    local r3 = throttle - dp - dr - dy   -- M3 BR CCW
    local r4 = throttle - dp + dr + dy   -- M4 BL CW

    -- anti-windup: shift all if any saturate
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

    self.rpm = {r1, r2, r3, r4}
    setMotorRPM(self.motors[1], r1)
    setMotorRPM(self.motors[2], r2)
    setMotorRPM(self.motors[3], r3)
    setMotorRPM(self.motors[4], r4)
end

function Mixer:allStop()
    for i = 1, 4 do
        setMotorRPM(self.motors[i], 0)
        self.rpm[i] = 0
    end
end

function Mixer:status()
    local s = ""
    local labels = {"FL","FR","BR","BL"}
    for i = 1, 4 do
        local ok = self.motors[i] and "OK" or "--"
        s = s .. string.format("%s:%s(%3d) ", labels[i], ok, self.rpm[i])
    end
    return s
end

return Mixer
