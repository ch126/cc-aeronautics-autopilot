-- =============================================================
--  quad/main.lua   (GUI version)
--  Quadrotor flight controller with TUI
-- =============================================================

-- ── pre-flight check ───────────────────────────────────────────
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.clear()
term.setCursorPos(1, 1)

if not term.isColor() then
    print("ERROR: Advanced Computer required!")
    print("This program needs colors (Advanced Computer).")
    return
end

-- safe loader: show error on screen and halt if dofile fails
local function safe_load(path)
    local ok, result = pcall(dofile, path)
    if not ok then
        term.setTextColor(colors.red)
        print("LOAD ERROR: " .. path)
        term.setTextColor(colors.yellow)
        print(tostring(result))
        term.setTextColor(colors.white)
        print("")
        print("Press any key to exit.")
        os.pullEvent("key")
        error("load failed: " .. path)
    end
    return result
end

print("Loading quad FC...")
local C    = safe_load("/quad/config.lua")
local IMU  = safe_load("/quad/imu.lua")
local Mix  = safe_load("/quad/mixer.lua")
local Ctrl = safe_load("/quad/controller.lua")
local GUI  = safe_load("/quad/gui.lua")

-- globals
local imu_state   = {}
local running     = true
local ctrl_dt     = 1 / C.CTRL_HZ
local nav_dt      = 1 / C.NAV_HZ
local start_t     = os.clock()
local _nav_landing = false   -- nav_arrived 触发的自动降落标志

local imu   = IMU.new()
local mixer = Mix.new()
local ctrl  = Ctrl.new()
local gui   = GUI.new()

-- build status table for GUI
local function make_status()
    local s = imu_state
    return {
        armed      = ctrl.armed,
        mode       = ctrl.armed and "ARMED" or "IDLE",
        pitch      = s.pitch      or 0,
        roll       = s.roll       or 0,
        yaw        = s.yaw        or 0,
        alt        = s.altitude   or 0,
        climb      = s.climb_rate or 0,
        target_alt = ctrl.target_alt or 0,
        target_yaw = ctrl.target_yaw or 0,
        x          = s.x,
        z          = s.z,
        target_x   = ctrl.target_x,
        target_z   = ctrl.target_z,
        gps_locked = (ctrl.target_x ~= nil) and (s.x ~= nil),
        sensors    = imu:status(),
        elapsed    = os.clock() - start_t,
        throttle   = ctrl.throttle_out or 0,
        rpm_max    = C.RPM_MAX,
    }
end

local function make_motors()
    local r = mixer.rpm or {0,0,0,0}
    return {
        fl       = r[1] or 0,
        fr       = r[2] or 0,
        br       = r[3] or 0,
        bl       = r[4] or 0,
        throttle = ctrl.throttle_out or 0,
    }
end

-- ctrl loop 20Hz：内环 + 外环合并，减少调度延迟
local function ctrlLoop()
    local last_t    = os.clock()
    local outer_acc = 0   -- 外环累计时间
    local OUTER_DT  = 0.25  -- 外环 4Hz
    while running do
        local now = os.clock()
        local dt  = math.max(0.001, math.min(now - last_t, 0.2))
        last_t    = now
        imu:read(dt)
        imu_state = imu
        if ctrl.armed then
            outer_acc = outer_acc + dt
            if outer_acc >= OUTER_DT then
                ctrl:updateOuter(imu_state, outer_acc)
                outer_acc = 0
                -- 到达检测：控制器切换到悬停后通知用户
                if ctrl.arrived then
                    ctrl.arrived = false
                    gui:log(string.format("Arrived! Holding (%.1f, %.1f)",
                        ctrl.target_x or 0, ctrl.target_z or 0), "OK")
                end
                -- navfollow 到达：自动降落
                if ctrl.nav_arrived then
                    ctrl.nav_arrived = false
                    gui:log("Nav arrived! Auto-landing...", "OK")
                    ctrl.target_alt = 0.3
                    -- 开一个 coroutine-style 延时降落（直接在 ctrlLoop 里 sleep 会阻塞整个环）
                    -- 改为设置标志，让 inputLoop 里单独处理
                    _nav_landing = true
                end
            end
            local po, ro, yo = ctrl:updateInner(imu_state, dt)
            mixer:mix(ctrl.throttle_out or C.RPM_HOVER, po, ro, yo)
        else
            outer_acc = 0
            mixer:allStop()
        end
        local sleep_t = ctrl_dt - (os.clock() - now)
        if sleep_t > 0.001 then os.sleep(sleep_t) end
    end
end

-- render loop ~5Hz
local function renderLoop()
    while running do
        if not gui.modal then
            gui:render(make_status(), make_motors())
        end
        os.sleep(0.2)
    end
end

-- GPS loop ~5Hz + 自动降落监控
local function gpsLoop()
    while running do
        imu:readGPS()
        -- nav_arrived 触发的自动降落：等高度下降后 disarm
        if _nav_landing and ctrl.armed then
            if (imu_state.altitude or 99) <= 0.5 then
                os.sleep(0.5)
                ctrl:disarm()
                _nav_landing = false
                gui:log("Landed and disarmed.", "OK")
            end
        end
        os.sleep(0.2)   -- 5Hz GPS 更新，gps.locate(0.1) 本身占 0.1s
    end
end

-- command handler
local function handle_cmd(line)
    line = line:match("^%s*(.-)%s*$")
    local parts = {}
    for w in line:gmatch("%S+") do parts[#parts+1] = w end
    local cmd = parts[1] or ""

    if cmd == "help" then
        gui:log("arm disarm hover goto navfollow alt yaw land pos motors quit", "INFO")
    elseif cmd == "arm" then
        local h = tonumber(parts[2]) or 5
        gui:log("Calibrating sensors...", "INFO")
        local bx, bz = imu:calibrate(8)
        gui:log(string.format("Bias vx=%.3f vz=%.3f", bx, bz), "INFO")
        imu.x = 0.0
        imu.z = 0.0
        imu._origin_x = nil   -- 重置 GPS 起飞原点（下一帧自动设定）
        imu._origin_z = nil
        imu.gps_ok    = false
        ctrl:arm(imu_state.altitude or 0, imu_state.yaw or 0)
        ctrl.target_alt = h
        -- 导航台在线时自动启用位置保持（以起飞点为原点目标）
        -- 即使未配置 NAV_BEACON_X，DR+yaw 修正也能提供基础位置保持
        if imu.nav_p then
            ctrl.target_x = 0.0
            ctrl.target_z = 0.0
            gui:log("Position hold: ON (DR" .. (C.NAV_BEACON_X and "+Nav)" or " only)"), "OK")
        end
        gui:log(string.format("Armed, target alt %.1f m", h), "OK")
    elseif cmd == "disarm" then
        ctrl:disarm()
        gui:log("Disarmed", "WARN")
    elseif cmd == "hover" then
        ctrl:hover(imu_state)
        gui:log(string.format("Hovering, alt %.1f", imu_state.altitude or 0), "OK")
    elseif cmd == "goto" then
        -- goto x z [alt]  —— 坐标相对起飞点（航位推算）
        local x   = tonumber(parts[2])
        local z   = tonumber(parts[3])
        local alt = tonumber(parts[4]) or ctrl.target_alt
        if x and z then
            ctrl.target_x     = x
            ctrl.target_z     = z
            ctrl.target_alt   = alt
            ctrl._goto_active = true   -- 触发到达检测
            ctrl.arrived      = false
            gui:log(string.format("Goto (%.1f, %.1f) alt %.1f", x, z, alt), "OK")
        else
            gui:log("Usage: goto <x> <z> [alt]", "WARN")
        end
    elseif cmd == "alt" then
        local h = tonumber(parts[2])
        if h then ctrl.target_alt = h
            gui:log(string.format("Target alt -> %.1f m", h), "OK")
        end
    elseif cmd == "yaw" then
        local y = tonumber(parts[2])
        if y then ctrl.target_yaw = y % 360
            gui:log(string.format("Target yaw -> %.1f deg", ctrl.target_yaw), "OK")
        end
    elseif cmd == "poshold" then
        -- 手动开启/关闭位置保持，以当前位置为目标
        if ctrl.target_x ~= nil then
            ctrl.target_x = nil
            ctrl.target_z = nil
            gui:log("Position hold: OFF", "WARN")
        else
            ctrl.target_x = imu_state.x or 0
            ctrl.target_z = imu_state.z or 0
            gui:log(string.format("Position hold: ON @ (%.2f, %.2f)", ctrl.target_x, ctrl.target_z), "OK")
        end
    elseif cmd == "pos" then
        local s = imu_state
        gui:log(string.format("P%.1f R%.1f Y%.1f Alt%.2f Clmb%.2f",
            s.pitch or 0, s.roll or 0, s.yaw or 0,
            s.altitude or 0, s.climb_rate or 0), "INFO")
        gui:log(string.format("x=%.2f z=%.2f vx=%.3f vz=%.3f tx=%s tz=%s gps=%s nav_lat=%.2f",
            s.x or 0, s.z or 0, s.vx or 0, s.vz or 0,
            ctrl.target_x and string.format("%.2f", ctrl.target_x) or "nil",
            ctrl.target_z and string.format("%.2f", ctrl.target_z) or "nil",
            imu.gps_ok and "OK" or "--",
            imu.nav_lateral or 0), "INFO")
        if ctrl.dbg then
            local d = ctrl.dbg
            gui:log(string.format("err(%.2f,%.2f) tv(%.2f,%.2f) rp=%.1f rr=%.1f i(%.2f,%.2f)",
                d.ex or 0, d.ez or 0, d.tvx or 0, d.tvz or 0,
                d.rp or 0, d.rr or 0, d.ix or 0, d.iz or 0), "INFO")
        end
        -- navfollow 到达进度
        if ctrl.nav_follow_speed then
            local da = ctrl.dbg_arrive
            if da then
                gui:log(string.format(
                    "navfollow: rel=%.1f was_behind=%s arrive=%.2f/%.2f",
                    da.rel, tostring(da.was_behind), da.acc, da.need), "INFO")
            end
        end
    elseif cmd == "motors" then
        local r = mixer.rpm or {0,0,0,0}
        local rf = mixer.rpm_filt or {0,0,0,0}
        gui:log(string.format("tgt FL%d FR%d BR%d BL%d",
            r[1] or 0, r[2] or 0, r[3] or 0, r[4] or 0), "INFO")
        gui:log(string.format("filt FL%d FR%d BR%d BL%d",
            math.floor(rf[1] or 0), math.floor(rf[2] or 0),
            math.floor(rf[3] or 0), math.floor(rf[4] or 0)), "INFO")
    elseif cmd == "tilt" then
        -- tilt <pitch> <roll>：直接注入目标倾斜角（度），测试混控链路
        -- 例：tilt 5 0 → 应前倾，FL/BL转速提高，FR/BR降低
        local p = tonumber(parts[2]) or 5
        local r = tonumber(parts[3]) or 0
        if ctrl.armed then
            ctrl.target_pitch = p
            ctrl.target_roll  = r
            gui:log(string.format("Injected pitch=%.1f roll=%.1f  watch motors cmd", p, r), "WARN")
        else
            gui:log("Arm first", "WARN")
        end
    elseif cmd == "gpstest" then
        -- 诊断 GPS 定位问题：列出 modem 并执行较长超时的 gps.locate()
        gui:log("=== GPS DIAG ===", "INFO")
        gui:log("注意: SubLevel上gps.locate()返回本地坐标非世界坐标", "WARN")
        gui:log("USE_GPS_LOCATE=" .. tostring(C.USE_GPS_LOCATE), "INFO")
        -- 列出所有 modem
        local found_wireless = false
        peripheral.find("modem", function(name, m)
            local wl = m.isWireless and m.isWireless() or false
            local ch = wl and (m.isOpen and m.isOpen(65534)) or false
            gui:log(string.format("modem %s wireless=%s ch65534=%s", name, tostring(wl), tostring(ch)), "INFO")
            if wl then
                found_wireless = true
                if not ch then
                    m.open(65534)
                    gui:log("  -> opened ch 65534", "WARN")
                end
            end
        end)
        if not found_wireless then
            gui:log("NO wireless modem found!", "WARN")
        end
        -- 尝试定位（2秒超时，更宽松）
        gui:log("Locating (2s timeout)...", "INFO")
        local wx, wy, wz = gps.locate(2)
        if wx then
            gui:log(string.format("GPS OK: %.1f, %.1f, %.1f", wx, wy, wz), "OK")
        else
            gui:log("GPS FAIL: returned nil", "WARN")
            gui:log("需要在世界中放置 >=3 个 GPS 主机电脑并运行 gps host", "WARN")
        end
    elseif cmd == "navfollow" then
        -- navfollow [speed]  — 以固定速度跟着导航台指向的目标飞
        -- 用 hover 或 poshold 停止
        if not ctrl.armed then
            gui:log("Arm first", "WARN")
        elseif not imu.nav_p then
            gui:log("No navigation_table peripheral found", "WARN")
        else
            local spd = tonumber(parts[2]) or 1.5
            spd = math.max(0.2, math.min(spd, C.POS_MAX_VEL))
            ctrl.nav_follow_speed = spd
            ctrl._nav_arrive_acc  = 0       -- 重置到达计时
            ctrl._nav_follow_time = 0       -- 重置飞行计时
            ctrl.nav_arrived      = false
            ctrl.target_x = nil   -- 关闭位置定点，避免冲突
            ctrl.target_z = nil
            gui:log(string.format("Nav follow ON, speed=%.1f m/s  (hover to stop)", spd), "OK")
        end
    elseif cmd == "land" then
        gui:log("Landing...", "WARN")
        ctrl.target_alt = 0.3
        os.sleep(4)
        ctrl:disarm()
        gui:log("Landed", "OK")
    elseif cmd == "quit" then
        ctrl:disarm()
        running = false
        gui:log("Quit", "WARN")
    elseif cmd ~= "" then
        gui:log("Unknown: " .. cmd, "WARN")
    end
end

-- button handler
local function handle_button(action)
    if action == "arm" then
        local res = gui:dialog("ARM", {
            { label="Target altitude (m):", default="5" },
        })
        if res then
            local h = tonumber(res[1]) or 5
            gui:log("Calibrating sensors...", "INFO")
            local bx, bz = imu:calibrate(8)
            gui:log(string.format("Bias vx=%.3f vz=%.3f", bx, bz), "INFO")
            imu.x = 0.0   -- 重置航位推算原点
            imu.z = 0.0
            ctrl:arm(imu_state.altitude or 0, imu_state.yaw or 0)
            ctrl.target_alt = h
            gui:log(string.format("Armed, target alt %.1f m", h), "OK")
        end
    elseif action == "disarm" then
        ctrl:disarm()
        gui:log("Disarmed", "WARN")
    elseif action == "hover" then
        ctrl:hover(imu_state)
        gui:log("Hovering", "OK")
    elseif action == "land" then
        ctrl.target_alt = 0.3
        gui:log("Landing...", "WARN")
    elseif action == "goto" then
        local res = gui:dialog("GOTO", {
            { label="X:", default="0" },
            { label="Z:", default="0" },
            { label="Alt (m):", default=tostring(math.floor(ctrl.target_alt or 5)) },
        })
        if res then
            local x   = tonumber(res[1])
            local z   = tonumber(res[2])
            local alt = tonumber(res[3]) or ctrl.target_alt
            if x and z then
                ctrl.target_x     = x
                ctrl.target_z     = z
                ctrl.target_alt   = alt
                ctrl._goto_active = true
                ctrl.arrived      = false
                gui:log(string.format("Goto (%.1f,%.1f) alt %.1f", x, z, alt), "OK")
            end
        end
    elseif action == "help" then
        gui:log("arm disarm hover land goto alt yaw pos motors quit", "INFO")
    end
end

-- input loop
local function inputLoop()
    while running do
        local ev, p1, p2, p3 = os.pullEvent()
        if ev == "char" then
            gui.input_buf = gui.input_buf .. p1
            gui:drawInput()
        elseif ev == "key" then
            if p1 == keys.backspace then
                if #gui.input_buf > 0 then
                    gui.input_buf = gui.input_buf:sub(1, -2)
                    gui:drawInput()
                end
            elseif p1 == keys.enter then
                local line = gui.input_buf
                gui.input_buf = ""
                gui:drawInput()
                handle_cmd(line)
            end
        elseif ev == "mouse_click" then
            local action = gui:hitButton(p2, p3)
            if action then
                gui:drawButtons(action)
                handle_button(action)
                gui:drawButtons()
            end
        end
    end
end

-- main
gui:drawFrame()
gui:log("Quad FC started  CTRL_HZ=" .. C.CTRL_HZ, "OK")
gui:log("Motors: " .. mixer:status(), "INFO")
gui:drawInput()

parallel.waitForAny(ctrlLoop, renderLoop, gpsLoop, inputLoop)

mixer:allStop()
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.clear()
term.setCursorPos(1, 1)
print("Quad FC stopped.")
