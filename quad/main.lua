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
local imu_state = {}
local running   = true
local ctrl_dt   = 1 / C.CTRL_HZ
local nav_dt    = 1 / C.NAV_HZ
local start_t   = os.clock()

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

-- GPS loop ~1Hz (独立循环，避免阻塞控制环)
local function gpsLoop()
    while running do
        imu:readGPS()
        os.sleep(1.0)
    end
end

-- command handler
local function handle_cmd(line)
    line = line:match("^%s*(.-)%s*$")
    local parts = {}
    for w in line:gmatch("%S+") do parts[#parts+1] = w end
    local cmd = parts[1] or ""

    if cmd == "help" then
        gui:log("arm disarm hover goto alt yaw land pos motors quit", "INFO")
    elseif cmd == "arm" then
        local h = tonumber(parts[2]) or 5
        gui:log("Calibrating sensors...", "INFO")
        local bx, bz = imu:calibrate(8)
        gui:log(string.format("Bias vx=%.3f vz=%.3f", bx, bz), "INFO")
        imu.x = 0.0
        imu.z = 0.0
        ctrl:arm(imu_state.altitude or 0, imu_state.yaw or 0)
        ctrl.target_alt = h
        -- 导航台可用时自动启用位置保持（以起飞点为原点目标）
        if imu.nav_p and C.NAV_BEACON_X then
            ctrl.target_x = 0.0
            ctrl.target_z = 0.0
            gui:log("Nav position hold: ON", "OK")
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
            ctrl.target_x   = x
            ctrl.target_z   = z
            ctrl.target_alt = alt
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
    elseif cmd == "pos" then
        local s = imu_state
        gui:log(string.format("P%.1f R%.1f Y%.1f Alt%.2f Clmb%.2f",
            s.pitch or 0, s.roll or 0, s.yaw or 0,
            s.altitude or 0, s.climb_rate or 0), "INFO")
    elseif cmd == "motors" then
        local r = mixer.rpm or {0,0,0,0}
        gui:log(string.format("FL%d FR%d BR%d BL%d",
            r[1] or 0, r[2] or 0, r[3] or 0, r[4] or 0), "INFO")
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
                ctrl.target_x   = x
                ctrl.target_z   = z
                ctrl.target_alt = alt
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
