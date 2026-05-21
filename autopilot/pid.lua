-- =============================================================
--  autopilot/pid.lua
--  通用 PID 控制器（支持独立三轴 / 单轴实例化）
-- =============================================================

local PID = {}
PID.__index = PID

---创建一个新的 PID 实例
---@param cfg table  { kp, ki, kd, integral_max, output_max }
function PID.new(cfg)
    return setmetatable({
        kp           = cfg.kp           or 1.0,
        ki           = cfg.ki           or 0.0,
        kd           = cfg.kd           or 0.0,
        integral_max = cfg.integral_max or math.huge,
        output_max   = cfg.output_max   or math.huge,
        _integral    = 0.0,
        _last_error  = 0.0,
        _initialized = false,
    }, PID)
end

---重置积分与微分状态
function PID:reset()
    self._integral    = 0.0
    self._last_error  = 0.0
    self._initialized = false
end

---计算 PID 输出
---@param setpoint number  目标值
---@param measured  number  当前测量值
---@param dt        number  时间步长 (秒)
---@return number output   控制量
function PID:compute(setpoint, measured, dt)
    if dt <= 0 then return 0 end

    local error = setpoint - measured

    -- 积分项（梯形积分，防积分饱和）
    self._integral = self._integral + error * dt
    -- 积分限幅
    self._integral = math.max(-self.integral_max,
                     math.min( self.integral_max, self._integral))

    -- 微分项（后向差分；首次调用无微分冲击）
    local derivative = 0.0
    if self._initialized then
        derivative = (error - self._last_error) / dt
    end
    self._initialized = true
    self._last_error  = error

    local output = self.kp * error
                 + self.ki * self._integral
                 + self.kd * derivative

    -- 输出限幅
    output = math.max(-self.output_max, math.min(self.output_max, output))
    return output
end

---动态修改增益（用于在线调参）
function PID:tune(kp, ki, kd)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self:reset()
end

-- =============================================================
--  三轴 PID 包装器（X、Y、Z 各自独立）
-- =============================================================
local PID3 = {}
PID3.__index = PID3

function PID3.new(cfg_h, cfg_v)
    return setmetatable({
        x = PID.new(cfg_h),
        y = PID.new(cfg_v),
        z = PID.new(cfg_h),
    }, PID3)
end

---三轴同时计算
---@param target  table { x, y, z }  目标位置
---@param current table { x, y, z }  当前位置
---@param dt      number
---@return table { x, y, z }  三轴控制量
function PID3:compute(target, current, dt)
    return {
        x = self.x:compute(target.x, current.x, dt),
        y = self.y:compute(target.y, current.y, dt),
        z = self.z:compute(target.z, current.z, dt),
    }
end

function PID3:reset()
    self.x:reset()
    self.y:reset()
    self.z:reset()
end

return {
    PID  = PID,
    PID3 = PID3,
}
