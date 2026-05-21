-- =============================================================
--  autopilot/nav.lua
--  导航管理器
--
--  职责：
--    1. 维护航点队列（waypoint queue）
--    2. 每帧调用 PID 计算控制量并写入飞艇外设
--    3. 协调避障模块修正目标位置
--    4. 暴露状态供 main.lua 展示
--
--  Create:Aeronautics 飞艇控制器（Helm）外设 API（1.21.1）：
--    helm.setThrottle(forward, up, right)   -- 各轴 [-1,1]
--    helm.setYawTarget(degrees)             -- 设置偏航目标角
--    helm.getVelocity()                     -- 返回 {x,y,z}
--    helm.getPosition()                     -- 返回 {x,y,z}
--    helm.getYaw()                          -- 返回当前偏航角（度）
--    helm.getAssemblyInfo()                 -- 返回飞艇信息
--    helm.disassemble()                     -- 解散飞艇
-- =============================================================

local Vec3     = require("autopilot.vec3")
local PIDMod   = require("autopilot.pid")
local Obstacle = require("autopilot.obstacle")
local Config   = require("autopilot.config")

local Nav = {}
Nav.__index = Nav

-- ── 状态枚举 ──────────────────────────────────────────────────
Nav.STATE = {
    IDLE      = "IDLE",
    NAVIGATING= "NAVIGATING",
    AVOIDING  = "AVOIDING",
    ARRIVED   = "ARRIVED",
    ERROR     = "ERROR",
}

-- ── 构造 ──────────────────────────────────────────────────────
function Nav.new()
    -- 查找外设
    local helm  = peripheral.find("create_aeronautics:helm")
               or peripheral.find("airshipController")  -- 兼容旧版命名
    local radar = Config.HAS_RADAR
                  and peripheral.find("neuralInterface")
                  or nil

    if not helm then
        error("[NAV] 未找到 Create:Aeronautics 飞艇控制器外设！\n"
            .."请确认:\n"
            .."  1. 飞艇已组装且电脑已连接到 Helm\n"
            .."  2. config.lua 中 HELM_NAME 正确\n"
            .."  3. NeoForge 1.21.1 + CC:Tweaked + Create:Aeronautics 均已安装")
    end

    local pid3 = PIDMod.PID3.new(Config.PID_H, Config.PID_V)
    local pidY = PIDMod.PID.new(Config.PID_YAW)
    local obs  = Obstacle.new(radar)

    return setmetatable({
        helm       = helm,
        pid3       = pid3,
        pid_yaw    = pidY,
        obstacle   = obs,

        -- 航点队列
        waypoints  = {},
        wp_index   = 1,

        -- 当前状态
        state      = Nav.STATE.IDLE,
        status_msg = "待机",

        -- 每帧缓存
        pos        = Vec3.new(0,0,0),
        velocity   = Vec3.new(0,0,0),
        yaw        = 0.0,
        tick       = 0,

        -- 统计
        total_dist  = 0.0,
        elapsed_sec = 0.0,
    }, Nav)
end

-- ── 航点管理 ──────────────────────────────────────────────────

---添加单个航点
function Nav:addWaypoint(x, y, z)
    table.insert(self.waypoints, Vec3.new(x, y, z))
end

---清空航点队列
function Nav:clearWaypoints()
    self.waypoints = {}
    self.wp_index  = 1
    self:_stop()
    self.state = Nav.STATE.IDLE
    self.status_msg = "航点已清空"
end

---开始导航（从第一个航点出发）
function Nav:start()
    if #self.waypoints == 0 then
        self.status_msg = "错误：没有设置航点"
        self.state = Nav.STATE.ERROR
        return
    end
    self.wp_index = 1
    self.pid3:reset()
    self.pid_yaw:reset()
    self.state      = Nav.STATE.NAVIGATING
    self.status_msg = string.format("导航至航点 1/%d", #self.waypoints)
end

---紧急停止
function Nav:stop()
    self:_stop()
    self.state      = Nav.STATE.IDLE
    self.status_msg = "已手动停止"
end

-- ── 主更新循环（每 TICK_RATE 秒调用一次）────────────────────
---@param dt number  时间步长（秒）
function Nav:update(dt)
    self.tick          = self.tick + 1
    self.elapsed_sec   = self.elapsed_sec + dt

    if self.state ~= Nav.STATE.NAVIGATING
    and self.state ~= Nav.STATE.AVOIDING then
        return
    end

    -- 1. 读取当前状态
    self:_readState()

    -- 2. 获取当前目标航点
    local target = self.waypoints[self.wp_index]
    if not target then
        self:_stop()
        self.state      = Nav.STATE.ARRIVED
        self.status_msg = "所有航点已到达！"
        return
    end

    -- 3. 到达判断
    local dist = (self.pos - target):length()
    if dist <= Config.ARRIVAL_RADIUS then
        self.wp_index = self.wp_index + 1
        if self.wp_index > #self.waypoints then
            self:_stop()
            self.state      = Nav.STATE.ARRIVED
            self.status_msg = string.format("目标到达！总飞行距离 %.1f 格，耗时 %.1f 秒",
                                            self.total_dist, self.elapsed_sec)
        else
            self.status_msg = string.format("到达中间航点，前往 %d/%d",
                                            self.wp_index, #self.waypoints)
            self.pid3:reset()
            self.pid_yaw:reset()
        end
        return
    end

    -- 4. 避障势场修正
    local virtual_target, has_obs = self.obstacle:compute(
        self.pos, target, self.tick)

    if has_obs then
        self.state      = Nav.STATE.AVOIDING
        self.status_msg = string.format(
            "避障中 → 航点 %d/%d  dist=%.1f",
            self.wp_index, #self.waypoints, dist)
    else
        self.state      = Nav.STATE.NAVIGATING
        self.status_msg = string.format(
            "导航至航点 %d/%d  dist=%.1f 格",
            self.wp_index, #self.waypoints, dist)
    end

    -- 5. PID 控制量计算
    self:_controlStep(virtual_target, target, dt, dist)

    -- 6. 累计里程
    if self._last_pos then
        self.total_dist = self.total_dist + (self.pos - self._last_pos):length()
    end
    self._last_pos = self.pos:clone()
end

-- ── 内部：读取飞艇传感器 ─────────────────────────────────────
function Nav:_readState()
    local ok, p = pcall(function() return self.helm.getPosition() end)
    if ok and p then
        self.pos = Vec3.new(p.x, p.y, p.z)
    end

    local ok2, v = pcall(function() return self.helm.getVelocity() end)
    if ok2 and v then
        self.velocity = Vec3.new(v.x, v.y, v.z)
    end

    local ok3, y = pcall(function() return self.helm.getYaw() end)
    if ok3 and y then
        self.yaw = y
    end
end

-- ── 内部：PID → 油门写入 ─────────────────────────────────────
function Nav:_controlStep(vtarget, real_target, dt, dist)
    local cfg = Config
    local pos = self.pos

    -- ── 偏航控制（先对准再前进）──────────────────────────────
    local desired_yaw = pos:yawTo(vtarget)
    local yaw_err     = Vec3.normalizeAngle(desired_yaw - self.yaw)
    local yaw_out     = self.pid_yaw:compute(0, -yaw_err, dt)
    -- yaw_out > 0 → 右转，< 0 → 左转

    -- Create:Aeronautics setYaw 接口（部分版本）
    pcall(function()
        self.helm.setYawTarget(desired_yaw)
    end)

    -- ── 水平推力（仅当偏航对齐后施加，减少侧滑）────────────
    local abs_yaw_err = math.abs(yaw_err)
    local heading_ok  = abs_yaw_err < cfg.HEADING_THRESHOLD

    -- PID 三轴控制量（世界坐标系）
    local ctrl = self.pid3:compute(
        { x = vtarget.x, y = vtarget.y, z = vtarget.z },
        { x = pos.x,     y = pos.y,     z = pos.z     },
        dt
    )

    -- 将世界坐标控制量转换为飞艇本地坐标（前/右）
    local yaw_rad = math.rad(self.yaw)
    local fwd  =  ctrl.x * math.sin(yaw_rad) + ctrl.z * (-math.cos(yaw_rad))
    local right=  ctrl.x * math.cos(yaw_rad) + ctrl.z * math.sin(yaw_rad)
    local up   =  ctrl.y

    -- 速度限幅
    local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
    fwd   = clamp(fwd,   -cfg.MAX_THROTTLE, cfg.MAX_THROTTLE)
    right = clamp(right, -cfg.MAX_THROTTLE, cfg.MAX_THROTTLE)
    up    = clamp(up,    -cfg.MAX_THROTTLE, cfg.MAX_THROTTLE)

    -- 偏航未对齐时降低水平推力（避免乱飘）
    if not heading_ok then
        local factor = 1.0 - math.min(1.0, abs_yaw_err / 45.0)
        fwd   = fwd   * factor
        right = right * factor
    end

    -- 写入飞艇控制器
    -- Create:Aeronautics Helm API:
    --   setThrottle(forward, up, right)  或  setSail(f, u, r)
    pcall(function()
        self.helm.setThrottle(fwd, up, right)
    end)
    -- 兼容旧版
    pcall(function()
        self.helm.setSail(fwd, up, right)
    end)
end

-- ── 内部：停止推力 ────────────────────────────────────────────
function Nav:_stop()
    pcall(function() self.helm.setThrottle(0, 0, 0) end)
    pcall(function() self.helm.setSail(0, 0, 0) end)
    self.pid3:reset()
    self.pid_yaw:reset()
end

-- ── 状态查询 ──────────────────────────────────────────────────
function Nav:getStatus()
    return {
        state      = self.state,
        msg        = self.status_msg,
        pos        = self.pos,
        yaw        = self.yaw,
        velocity   = self.velocity,
        wp_current = self.wp_index,
        wp_total   = #self.waypoints,
        target     = self.waypoints[self.wp_index],
        dist       = self.waypoints[self.wp_index]
                     and (self.pos - self.waypoints[self.wp_index]):length()
                     or  0,
        total_dist = self.total_dist,
        elapsed    = self.elapsed_sec,
    }
end

return Nav
