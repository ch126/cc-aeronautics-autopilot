-- =============================================================
--  autopilot/obstacle.lua
--  人工势场法避障模块
--
--  工作原理：
--    合力 = 引力(目标) + 斥力(所有障碍物)
--    将合力方向作为修正后的"虚拟目标"传回 PID 导航
--
--  依赖：
--    vec3.lua   —— 三维向量
--    config.lua —— 避障参数
-- =============================================================

local Vec3   = require("autopilot.vec3")
local Config = require("autopilot.config")

local Obstacle = {}
Obstacle.__index = Obstacle

-- ── 内部工具 ──────────────────────────────────────────────────

local function log(msg)
    -- 简单打印，main.lua 可替换为带颜色的版本
    print("[OBST] " .. msg)
end

-- ── 雷达扫描（需要 Advanced Peripherals 或同类外设）──────────
-- 若无雷达，返回空列表
local function scanWithRadar(radar, pos, range)
    if not radar then return {} end
    local ok, result = pcall(function()
        return radar.scan(range)  -- 返回 { {x,y,z,name,...}, ... }
    end)
    if not ok or type(result) ~= "table" then return {} end
    -- 过滤出实心障碍（非空气、非飞艇自身方块）
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

-- ── 无雷达时的简易方块探测（仅用 GPS 高度推算地面）──────────
-- 在盲飞模式下仅维持最低安全高度，不做精细避障
local function groundGuard(pos, cfg)
    -- 假设 y=64 为默认地面，可通过 gps 或配置覆盖
    local ground_y = 64
    local min_y = ground_y + cfg.min_alt
    if pos.y < min_y then
        -- 返回一个"虚拟地面障碍"推飞艇向上
        return { Vec3.new(pos.x, ground_y, pos.z) }
    end
    return {}
end

-- ── 人工势场核心 ──────────────────────────────────────────────

---计算斥力合向量
---@param pos       Vec3   当前位置
---@param obstacles table  障碍物 Vec3 列表
---@param cfg       table  Config.OBSTACLE
---@return Vec3 repulsive_force
local function computeRepulsive(pos, obstacles, cfg)
    local total = Vec3.new(0, 0, 0)
    for _, obs in ipairs(obstacles) do
        local diff = pos - obs
        local dist = diff:length()
        if dist < cfg.repulse_range and dist > 0.1 then
            -- 斥力大小 = gain * (1/d - 1/d0)^2 / d^2  （标准势场公式简化版）
            local mag = cfg.repulse_gain
                      * (1.0/dist - 1.0/cfg.repulse_range)
                      / (dist * dist)
            total = total + diff:normalize() * mag
        end
    end
    return total
end

---计算引力向量（归一化）
---@param pos    Vec3
---@param target Vec3
---@param cfg    table
---@return Vec3 attractive_force
local function computeAttractive(pos, target, cfg)
    local diff = target - pos
    local dist = diff:length()
    if dist < 0.01 then return Vec3.new(0,0,0) end
    -- 线性引力（距离越远引力越大，但不超过 attract_gain）
    local mag = math.min(cfg.attract_gain, dist * 0.1)
    return diff:normalize() * mag
end

-- ── 公开 API ──────────────────────────────────────────────────

function Obstacle.new(radar_peripheral)
    return setmetatable({
        radar = radar_peripheral,
        cfg   = Config.OBSTACLE,
        -- 障碍物记忆缓存（避免每帧全扫，每 N 帧刷新一次）
        _cache      = {},
        _cache_tick = 0,
        _refresh_interval = 10,  -- 每 10 帧刷新一次雷达
    }, Obstacle)
end

---主接口：给定当前位置与目标，返回"势场修正后的虚拟目标位置"
---@param pos    Vec3  飞艇当前位置
---@param target Vec3  期望目标位置
---@param tick   number 当前帧号（用于缓存节流）
---@return Vec3  virtual_target  修正后的目标（传入 PID 计算）
---@return boolean has_obstacle  是否检测到障碍
function Obstacle:compute(pos, target, tick)
    local cfg = self.cfg

    -- 刷新障碍缓存
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

    -- 计算势场合力
    local F_att = computeAttractive(pos, target, cfg)
    local F_rep = computeRepulsive(pos, obstacles, cfg)
    local F_total = F_att + F_rep

    -- 将合力方向映射为虚拟目标（距当前位置固定步长）
    local step = 8.0
    local dir  = F_total:normalize()
    local virtual_target = pos + dir * step

    -- 垂直方向：若有障碍且合力 Y 分量朝下，强制抬升
    if virtual_target.y < pos.y - 1 then
        virtual_target.y = pos.y + cfg.alt_step
    end

    -- 确保不低于最低安全高度
    virtual_target.y = math.max(virtual_target.y, 64 + cfg.min_alt)

    return virtual_target, true
end

---手动更新地面高度（由外部 GPS / 高度计传入）
function Obstacle:setGroundY(y)
    self._ground_y = y
end

return Obstacle
