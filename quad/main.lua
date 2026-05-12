-- =============================================================
--  quad/main.lua
--  四旋翼飞控主程序
--  并行运行：控制环(50Hz) + 导航环(10Hz) + 输入监听
-- =============================================================

dofile("/quad/config.lua")  -- 预载以便后续 dofile 缓存命中
local C    = dofile("/quad/config.lua")
local IMU  = dofile("/quad/imu.lua")
local Mix  = dofile("/quad/mixer.lua")
local Ctrl = dofile("/quad/controller.lua")

-- ============ 状态 ============
local imu_state = {}   -- 最新 IMU 数据
local running   = true
local ctrl_dt   = 1 / C.CTRL_HZ
local nav_dt    = 1 / C.NAV_HZ

-- ============ 初始化 ============
local imu   = IMU.new()
local mixer = Mix.new()
local ctrl  = Ctrl.new()

print("[QUAD] 电机状态: " .. mixer:status())
print("[QUAD] 输入 'arm <高度>' 解锁，'help' 查看命令")

-- ============ 控制环 50Hz ============
local function ctrlLoop()
    local last_t = os.clock()
    while running do
        local now = os.clock()
        local dt  = now - last_t
        last_t    = now

        -- 读取传感器
        imu_state = imu:read(dt)

        if ctrl.armed then
            -- 内环：姿态+速率
            local po, ro, yo = ctrl:updateInner(imu_state, dt)
            -- 混控 + 输出
            mixer:mix(ctrl.throttle_out or C.RPM_HOVER, po, ro, yo)
        else
            mixer:allStop()
        end

        -- 精确等待到下一个控制周期
        local sleep_t = ctrl_dt - (os.clock() - now)
        if sleep_t > 0.001 then os.sleep(sleep_t) end
    end
end

-- ============ 导航环 10Hz ============
local function navLoop()
    local last_t = os.clock()
    while running do
        local now = os.clock()
        local dt  = now - last_t
        last_t    = now

        ctrl:updateOuter(imu_state, dt)

        local sleep_t = nav_dt - (os.clock() - now)
        if sleep_t > 0.001 then os.sleep(sleep_t) end
    end
end

-- ============ 帮助 ============
local function printHelp()
    print("  arm [高度]        解锁并起飞到指定高度（默认5m）")
    print("  disarm            加锁（立即停转）")
    print("  hover             悬停在当前位置")
    print("  goto <x> <z> [alt]  飞向坐标（需要GPS）")
    print("  alt <高度>        改变目标高度")
    print("  yaw <角度>        改变目标偏航")
    print("  pos               显示当前位置/姿态")
    print("  motors            显示电机转速")
    print("  land              降落并加锁")
    print("  quit              退出程序")
end

-- ============ 降落流程 ============
local function doLand()
    print("[QUAD] 开始降落...")
    ctrl.target_alt = 0.3
    os.sleep(3)
    ctrl:disarm()
    print("[QUAD] 已降落并加锁")
end

-- ============ 输入环 ============
local function inputLoop()
    while running do
        io.write("> ")
        local line = io.read()
        if not line then os.sleep(0.1) end
        if line then
        line = line:match("^%s*(.-)%s*$")  -- trim
        local parts = {}
        for w in line:gmatch("%S+") do parts[#parts+1] = w end
        local cmd = parts[1] or ""

        if cmd == "help" then
            printHelp()

        elseif cmd == "arm" then
            local h = tonumber(parts[2]) or 5
            ctrl:arm(imu_state.alt, imu_state.yaw)
            ctrl.target_alt = h
            print(string.format("[QUAD] 解锁，目标高度 %.1f m", h))

        elseif cmd == "disarm" then
            ctrl:disarm()
            print("[QUAD] 已加锁")

        elseif cmd == "hover" then
            ctrl:hover(imu_state)
            print("[QUAD] 悬停")

        elseif cmd == "goto" then
            if not imu_state.x then
                print("[QUAD] 无GPS，无法使用 goto")
            else
                local x   = tonumber(parts[2])
                local z   = tonumber(parts[3])
                local alt = tonumber(parts[4]) or ctrl.target_alt
                if x and z then
                    ctrl.target_x   = x
                    ctrl.target_z   = z
                    ctrl.target_alt = alt
                    print(string.format("[QUAD] 飞往 (%.1f, %.1f) 高度 %.1f", x, z, alt))
                else
                    print("用法: goto <x> <z> [alt]")
                end
            end

        elseif cmd == "alt" then
            local h = tonumber(parts[2])
            if h then
                ctrl.target_alt = h
                print(string.format("[QUAD] 目标高度 → %.1f m", h))
            end

        elseif cmd == "yaw" then
            local y = tonumber(parts[2])
            if y then
                ctrl.target_yaw = y % 360
                print(string.format("[QUAD] 目标偏航 → %.1f°", ctrl.target_yaw))
            end

        elseif cmd == "pos" then
            local s = imu_state
            if s.pitch then
                print(string.format(
                    "姿态: P=%.1f° R=%.1f° Y=%.1f°  高度: %.2fm  爬升率: %.2fm/s",
                    s.pitch or 0, s.roll or 0, s.yaw or 0,
                    s.alt   or 0, s.climb or 0))
                if s.x then
                    print(string.format("位置: X=%.2f  Z=%.2f", s.x, s.z))
                end
            else
                print("[QUAD] IMU 尚未就绪")
            end

        elseif cmd == "motors" then
            print("[QUAD] " .. mixer:status())

        elseif cmd == "land" then
            doLand()

        elseif cmd == "quit" then
            ctrl:disarm()
            running = false
            print("[QUAD] 退出")

        elseif cmd ~= "" then
            print("未知命令 '" .. cmd .. "'，输入 help 查看帮助")
        end
        end  -- if line
    end
end

-- ============ 主入口 ============
print("[QUAD] 启动飞控，CTRL_HZ=" .. C.CTRL_HZ .. " NAV_HZ=" .. C.NAV_HZ)
parallel.waitForAny(ctrlLoop, navLoop, inputLoop)
mixer:allStop()
print("[QUAD] 程序结束")
