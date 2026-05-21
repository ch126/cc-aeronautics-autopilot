-- =============================================================
--  autopilot/obstacle.lua

--

  -- = () + ()
  -- "" PID
--

  -- vec3.lua
  -- config.lua
-- =============================================================

local Vec3   = require("autopilot.vec3")
local Config = require("autopilot.config")

local Obstacle = {}
Obstacle.__index = Obstacle



local function log(msg)
  -- main.lua
    print("[OBST] " .. msg)
end

  -- Advanced Peripherals

local function scanWithRadar(radar, pos, range)
    if not radar then return {} end
    local ok, result = pcall(function()
        return radar.scan(range)  -- { {x,y,z,name,...}, ... }
    end)
    if not ok or type(result) ~= "table" then return {} end

    local obstacles = {}
    for _, entry in ipairs(result) do
        if entry.name and entry.name ~= "minecraft:air" then
            table.insert(obstacles, Vec3.new(
                pos.x + (entry.x or 0),
                pos.y + (entry.y or 0),
                pos.z + (entry.z or 0)
            ))
        end
    end
    return obstacles
end

  -- GPS

local function groundGuard(pos, cfg)
  -- y=64  gps
    local ground_y = 64
    local min_y = ground_y + cfg.min_alt
    if pos.y < min_y then
  -- ""
        return { Vec3.new(pos.x, ground_y, pos.z) }
    end
    return {}
end



  -- -
  -- -@param pos       Vec3
  -- -@param obstacles table   Vec3
---@param cfg       table  Config.OBSTACLE
---@return Vec3 repulsive_force
local function computeRepulsive(pos, obstacles, cfg)
    local total = Vec3.new(0, 0, 0)
    for _, obs in ipairs(obstacles) do
        local diff = pos - obs
        local dist = diff:length()
        if dist < cfg.repulse_range and dist > 0.1 then
  -- = gain * (1/d - 1/d0)^2 / d^2
            local mag = cfg.repulse_gain
                      * (1.0/dist - 1.0/cfg.repulse_range)
                      / (dist * dist)
            total = total + diff:normalize() * mag
        end
    end
    return total
end

  -- -
---@param pos    Vec3
---@param target Vec3
---@param cfg    table
---@return Vec3 attractive_force
local function computeAttractive(pos, target, cfg)
    local diff = target - pos
    local dist = diff:length()
    if dist < 0.01 then return Vec3.new(0,0,0) end
  -- attract_gain
    local mag = math.min(cfg.attract_gain, dist * 0.1)
    return diff:normalize() * mag
end

  -- API

function Obstacle.new(radar_peripheral)
    return setmetatable({
        radar = radar_peripheral,
        cfg   = Config.OBSTACLE,
  -- N
        _cache      = {},
        _cache_tick = 0,
        _refresh_interval = 10,  -- 10
    }, Obstacle)
end

  -- -""
  -- -@param pos    Vec3
  -- -@param target Vec3
  -- -@param tick   number
  -- -@return Vec3  virtual_target   PID
  -- -@return boolean has_obstacle
function Obstacle:compute(pos, target, tick)
    local cfg = self.cfg


    if tick - self._cache_tick >= self._refresh_interval then
        self._cache_tick = tick
        if self.radar then
            self._cache = scanWithRadar(self.radar, pos, cfg.detect_range)
        else
            self._cache = groundGuard(pos, cfg)
        end
    end

    local obstacles = self._cache
    local has_obstacle = #obstacles > 0

    if not has_obstacle then
        return target, false
    end


    local F_att = computeAttractive(pos, target, cfg)
    local F_rep = computeRepulsive(pos, obstacles, cfg)
    local F_total = F_att + F_rep


    local step = 8.0
    local dir  = F_total:normalize()
    local virtual_target = pos + dir * step

  -- Y
    if virtual_target.y < pos.y - 1 then
        virtual_target.y = pos.y + cfg.alt_step
    end


    virtual_target.y = math.max(virtual_target.y, 64 + cfg.min_alt)

    return virtual_target, true
end

  -- - GPS /
function Obstacle:setGroundY(y)
    self._ground_y = y
end

return Obstacle
