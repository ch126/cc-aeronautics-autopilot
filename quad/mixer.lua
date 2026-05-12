-- =============================================================
--  quad/mixer.lua
--  X型四旋翼混控矩阵 + 转速控制器驱动
--
--  电机布局（俯视）：
--
--         前(+Z方向)
--    M1(↺)     M2(↻)
--      \         /
--       +---------+
--      /         \
--    M4(↻)     M3(↺)
--         后
--
--  混控公式：
--    M1 = throttle + pitch + roll_R - yaw   (左前, CCW)
--    M2 = throttle + pitch - roll_R + yaw   (右前, CW)  ← roll_R=roll右正
--    M3 = throttle - pitch - roll_R - yaw   (右后, CCW)
--    M4 = throttle - pitch + roll_R + yaw   (左后, CW)
--
--  注意：CW电机转速取负号存储，setSpeed 时取绝对值。
--  Create转速控制器通过 setTargetSpeed(rpm) 设置。
-- =============================================================

local C = dofile("/quad/config.lua")

local Mixer = {}
Mixer.__index = Mixer

-- 自动发现转速控制器，按 FL/FR/BR/BL 顺序分配
local function findMotors()
    local motors = {}
    local names = { C.MOTOR_FL, C.MOTOR_FR, C.MOTOR_BR, C.MOTOR_BL }
    local labels = { "FL", "FR", "BR", "BL" }

    -- 先用配置名查找
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

    -- 自动扫描补全缺失电机
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
                    motors[i] = nil  -- 缺失
                end
            end
        end
    end
    return motors
end

function Mixer.new()
    local self = setmetatable({}, Mixer)
    self.motors = findMotors()
    -- 当前各电机 RPM 指令（用于监控）
    self.rpm = {0, 0, 0, 0}
    return self
end

-- 设置单个电机 RPM（内部调用）
local function setMotorRPM(motor, rpm)
    if not motor then return end
    rpm = math.max(C.RPM_MIN, math.min(C.RPM_MAX, rpm))
    -- Create转速控制器 API
    local ok, err = pcall(motor.p.setTargetSpeed, math.floor(rpm + 0.5))
    if not ok then
        -- 尝试备用 API 名称
        pcall(motor.p.setSpeed, math.floor(rpm + 0.5))
    end
end

-- 主混控函数
-- throttle : 0..RPM_MAX  基础油门（悬停约 RPM_HOVER）
-- pitch_out: -1..1       俯仰指令（+前倾）
-- roll_out : -1..1       横滚指令（+右倾）
-- yaw_out  : -1..1       偏航指令（+右偏航，机头顺时针）
function Mixer:mix(throttle, pitch_out, roll_out, yaw_out)
    -- 将 -1..1 指令映射到 RPM 差量
    local half_range = (C.RPM_MAX - C.RPM_MIN) * 0.5
    local dp = pitch_out * half_range * 0.5
    local dr = roll_out  * half_range * 0.5
    local dy = yaw_out   * half_range * 0.3

    -- X型混控矩阵
    --        throttle  pitch  roll   yaw
    local r1 = throttle + dp  + dr  - dy   -- M1 左前 CCW
    local r2 = throttle + dp  - dr  + dy   -- M2 右前 CW
    local r3 = throttle - dp  - dr  - dy   -- M3 右后 CCW
    local r4 = throttle - dp  + dr  + dy   -- M4 左后 CW

    -- 防饱和：如果任何值超界，等比缩放（保持姿态比例）
    local max_r = math.max(r1, r2, r3, r4)
    local min_r = math.min(r1, r2, r3, r4)
    if max_r > C.RPM_MAX then
        local excess = max_r - C.RPM_MAX
        r1 = r1 - excess; r2 = r2 - excess
        r3 = r3 - excess; r4 = r4 - excess
    end
    if min_r < C.RPM_MIN then
        local deficit = C.RPM_MIN - min_r
        r1 = r1 + deficit; r2 = r2 + deficit
        r3 = r3 + deficit; r4 = r4 + deficit
    end

    self.rpm = {r1, r2, r3, r4}
    setMotorRPM(self.motors[1], r1)
    setMotorRPM(self.motors[2], r2)
    setMotorRPM(self.motors[3], r3)
    setMotorRPM(self.motors[4], r4)
end

-- 全部停转
function Mixer:allStop()
    for i = 1, 4 do
        setMotorRPM(self.motors[i], 0)
        self.rpm[i] = 0
    end
end

-- 电机状态字符串
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
