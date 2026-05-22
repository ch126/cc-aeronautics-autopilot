-- =============================================================
--  car/main.lua
--  自动驾驶汽车主程序
--
--  命令:
--    drive <X> <Z> [spd]   前往目标坐标
--    wp <X> <Z> [spd]      添加航点
--    route                 开始执行航点队列
--    clear                 清空队列并停车
--    pos                   显示当前位置
--    origin                重置坐标原点到当前位置
--    stop                  停车
--    scan                  扫描外设
--    rstest <side> <0-15>  直接测试红石输出
--    rstest off            关所有红石
--    manual                手动驾驶 (W/S=油门 A/D=转向 X=停 M=退出)
--    help / exit
-- =============================================================

local Nav    = dofile("/car/nav.lua")
local Config = dofile("/car/config.lua")

-- 日志
local function log(msg, level)
    local col = colors.white
    if     level == "OK"   then col = colors.lime
    elseif level == "WARN" then col = colors.yellow
    elseif level == "ERR"  then col = colors.red
    elseif level == "INFO" then col = colors.lightGray
    end
    term.setTextColor(col)
    print("[" .. (level or "LOG") .. "] " .. msg)
    term.setTextColor(colors.white)
end

-- 开机画面
term.clear(); term.setCursorPos(1,1)
term.setTextColor(colors.cyan)
print("+--------------------------------------------+")
print("|   Create:Aeronautics  Car Autopilot  v1.0  |")
print("+--------------------------------------------+")
term.setTextColor(colors.white)

local nav = Nav.new()
log("Sensors: " .. nav.sensors:status(), "INFO")
log("Origin set at current position. Type 'help'.", "INFO")

-- 手动驾驶
local function manualMode()
    local p = math.floor(Config.RS_MAX * 0.6)
    log("MANUAL: W/S=油门  A/D=转向  X=停车  M=退出", "WARN")
    local function allStop()
        for _, s in ipairs({"front","back","left","right","top","bottom"}) do
            rs.setAnalogOutput(s, 0)
        end
    end
    while true do
        local _, key = os.pullEvent("key")
        if     key == keys.w then
            rs.setAnalogOutput(Config.SIDE_THROTTLE_F, p)
            rs.setAnalogOutput(Config.SIDE_THROTTLE_B, 0)
        elseif key == keys.s then
            rs.setAnalogOutput(Config.SIDE_THROTTLE_F, 0)
            rs.setAnalogOutput(Config.SIDE_THROTTLE_B, p)
        elseif key == keys.a then
            rs.setAnalogOutput(Config.SIDE_STEER_L, p)
            rs.setAnalogOutput(Config.SIDE_STEER_R, 0)
        elseif key == keys.d then
            rs.setAnalogOutput(Config.SIDE_STEER_L, 0)
            rs.setAnalogOutput(Config.SIDE_STEER_R, p)
        elseif key == keys.x then allStop()
        elseif key == keys.m then
            allStop(); nav.mode = "IDLE"
            log("Manual mode exited", "OK"); return
        end
    end
end

-- 命令处理
local function handleCmd(line)
    local parts = {}
    for w in line:gmatch("%S+") do table.insert(parts, w) end
    if #parts == 0 then return true end
    local cmd = parts[1]:lower()

    if cmd == "help" then
        log("drive <X> <Z> [spd]   前往目标坐标", "INFO")
        log("wp <X> <Z> [spd]      添加航点到队列", "INFO")
        log("route                 开始执行队列", "INFO")
        log("clear                 清空队列并停车", "INFO")
        log("pos                   显示当前位置", "INFO")
        log("origin                重置坐标原点", "INFO")
        log("stop                  停车", "INFO")
        log("scan                  扫描外设", "INFO")
        log("rstest <side> <0-15>  测试红石", "INFO")
        log("manual                手动驾驶", "INFO")

    elseif cmd == "scan" then
        local names = peripheral.getNames()
        if #names == 0 then log("No peripherals", "WARN") else
            for _, n in ipairs(names) do
                log(n .. " -> " .. peripheral.getType(n), "INFO")
            end
        end

    elseif cmd == "pos" then
        local st = nav:getStatus()
        log(string.format("Pos: X=%.2f Z=%.2f  (%s)",
            st.x, st.z, st.gps_ok and "GPS" or "DR"), "INFO")
        log(string.format("Hdg=%.1f  Spd=%.2f m/s  Dist=%.0f blk",
            st.heading, st.speed, st.dr_dist), "INFO")
        if st.target_x then
            local dx = st.target_x - st.x
            local dz = st.target_z - st.z
            log(string.format("Target X=%.0f Z=%.0f  remain=%.1f blk",
                st.target_x, st.target_z, math.sqrt(dx*dx+dz*dz)), "INFO")
            if st.wp_total > 0 then
                log(string.format("Route WP %d/%d", st.wp_index, st.wp_total), "INFO")
            end
        end

    elseif cmd == "origin" then
        nav.sensors:setOrigin(); log("Origin reset", "OK")

    elseif cmd == "drive" then
        local x, z, spd = tonumber(parts[2]), tonumber(parts[3]), tonumber(parts[4])
        if not (x and z) then log("Usage: drive <X> <Z> [speed]","WARN"); return true end
        nav:driveTo(x, z, spd)
        log(string.format("Driving to X=%.0f Z=%.0f", x, z), "OK")

    elseif cmd == "wp" then
        local x, z, spd = tonumber(parts[2]), tonumber(parts[3]), tonumber(parts[4])
        if not (x and z) then log("Usage: wp <X> <Z> [speed]","WARN"); return true end
        nav:addWaypoint(x, z, spd)
        log(string.format("WP added X=%.0f Z=%.0f  (total %d)", x, z, #nav.waypoints), "OK")

    elseif cmd == "route" then
        if #nav.waypoints == 0 then log("No waypoints. Use: wp <X> <Z>","WARN")
        else nav:setRoute(nav.waypoints)
             log(string.format("Route started: %d wps", #nav.waypoints), "OK") end

    elseif cmd == "clear" then
        nav:clearRoute(); log("Cleared & stopped", "WARN")

    elseif cmd == "stop" then
        nav:stop(); log("Stopped", "WARN")

    elseif cmd == "rstest" then
        if parts[2] == "off" then
            for _, s in ipairs({"top","bottom","left","right","front","back"}) do
                rs.setAnalogOutput(s, 0) end
            log("All RS OFF", "WARN")
        else
            local side, val = parts[2], tonumber(parts[3]) or 15
            if not side then log("Usage: rstest <side> <0-15>","WARN")
            else
                rs.setAnalogOutput(side, math.floor(math.max(0,math.min(15,val))))
                log(string.format("RS %s = %d", side, val), "OK")
            end
        end

    elseif cmd == "manual" then
        nav:stop(); nav.mode = "MANUAL"; manualMode()

    elseif cmd == "exit" or cmd == "quit" then
        nav:stop(); return false
    else
        log("Unknown: " .. cmd, "WARN")
    end
    return true
end

-- 主循环：parallel 同时运行 nav 控制循环和命令输入
local running = true

local function navLoop()
    while running do
        nav:update(Config.TICK_RATE)
        os.sleep(Config.TICK_RATE)
    end
end

local function inputLoop()
    while running do
        -- 非 IDLE 时在提示符前显示简短状态
        local st = nav:getStatus()
        if st.mode ~= "IDLE" and st.mode ~= "MANUAL" then
            term.setTextColor(colors.cyan)
            io.write(string.format("[%s] X=%.0f Z=%.0f hdg=%.0f spd=%.1f\n",
                st.mode, st.x, st.z, st.heading, st.speed))
            term.setTextColor(colors.white)
        end
        term.write("> ")
        local line = read()
        if handleCmd(line) == false then running = false end
    end
end

parallel.waitForAny(navLoop, inputLoop)
nav:stop()
print("Car autopilot stopped.")
