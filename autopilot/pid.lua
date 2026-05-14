-- =============================================================
--  autopilot/pid.lua
  -- PID  /
-- =============================================================

local PID = {}
PID.__index = PID

  -- - PID
---@param cfg table  { kp, ki, kd, integral_max, output_max }
function PID.new(cfg)
    return setmetatable({
        kp           = cfg.kp           or 1.0,
        ki           = cfg.ki           or 0.0,
        kd           = cfg.kd           or 0.0,
        integral_max = cfg.integral_max or math.huge,
        output_max   = cfg.output_max   or math.huge,
        _integral    = 0.0,
        _last_meas   = 0.0,   -- D项对测量值求导，避免setpoint突变时derivative kick
        _initialized = false,
    }, PID)
end

  -- -
function PID:reset()
    self._integral    = 0.0
    self._last_meas   = 0.0
    self._initialized = false
end

  -- - PID
  -- -@param setpoint number
  -- -@param measured  number
  -- -@param dt        number   ()
  -- -@return number output
function PID:compute(setpoint, measured, dt)
    if dt <= 0 then return 0 end

    local error = setpoint - measured

    -- 积分（带anti-windup钳位）
    self._integral = self._integral + error * dt
    self._integral = math.max(-self.integral_max,
                     math.min( self.integral_max, self._integral))

    -- D项对测量值求导（非误差），避免setpoint突变产生derivative kick
    local derivative = 0.0
    if self._initialized then
        derivative = -(measured - self._last_meas) / dt
    end
    self._initialized = true
    self._last_meas   = measured

    local output = self.kp * error
                 + self.ki * self._integral
                 + self.kd * derivative

    output = math.max(-self.output_max, math.min(self.output_max, output))
    return output
end

  -- -
function PID:tune(kp, ki, kd)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self:reset()
end

-- =============================================================
  -- PID XYZ
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

  -- -
  -- -@param target  table { x, y, z }
  -- -@param current table { x, y, z }
---@param dt      number
  -- -@return table { x, y, z }
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
