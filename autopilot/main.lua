-- =============================================================
--  autopilot/main.lua  (GUI + Manual Control)
--  Modes:
--    AUTO   - PID autopilot to waypoints
--    MANUAL - direct keyboard control  (type "manual" to enter)
--  Keys in MANUAL mode:
--    W/S = forward/back   A/D = strafe left/right
--    R/F = up/down        Q/E = yaw left/right
--    X   = stop all       M   = exit manual mode
-- =============================================================

package.path = package.path .. ";/autopilot/?.lua;/?.lua"

local Nav    = require("autopilot.nav")
local GUI    = require("autopilot.gui")
local Config = require("autopilot.config")

-- ── Init ──────────────────────────────────────────────────────
local gui = GUI.new()
gui:drawFrame()
gui:log("Initializing...", "INFO")

local ok_nav, nav = pcall(Nav.new)
if not ok_nav then
    gui:log("Helm not found: " .. tostring(nav), "ERR")
    gui:log("Type 'scan' to list peripherals.", "WARN")
    nav = nil
else
    gui:log("Helm connected. Ready!", "OK")
    if nav.helm._name then
        gui:log("Peripheral: " .. nav.helm._name, "INFO")
    end
end

-- ── Mode state ────────────────────────────────────────────────
local MODE = "AUTO"   -- "AUTO" | "MANUAL"
-- Manual throttle state (each axis -1..1)
local manual_fwd   = 0
local manual_side  = 0
local manual_up    = 0
local manual_yaw   = 0
local MANUAL_STEP  = 0.25   -- throttle increment per keypress

-- ── Safe accessors ────────────────────────────────────────────
local function safe_status()
    if nav then
        local s = nav:getStatus()
        if MODE == "MANUAL" then
            s.state = "MANUAL"
            s.msg   = string.format(
                "fwd:%.2f side:%.2f up:%.2f yaw:%.2f",
                manual_fwd, manual_side, manual_up, manual_yaw)
        end
        return s
    end
    local Vec3 = require("autopilot.vec3")
    return {
        state="ERROR", msg="Helm not connected",
        pos=Vec3.new(0,0,0), velocity=Vec3.new(0,0,0), yaw=0,
        wp_current=0, wp_total=0, target=nil, dist=0,
        total_dist=0, elapsed=0,
    }
end

local function safe_waypoints()
    if nav then return nav.waypoints, nav.wp_index end
    return {}, 1
end

-- ── Manual control: apply current throttles ───────────────────
local function apply_manual()
    if not nav then return end
    nav.helm.setThrottle(manual_fwd, manual_up, manual_side)
    nav.helm.setYawTarget(nav.pos.yaw or 0 + manual_yaw * 5)
end

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end

-- ── Manual key handler ────────────────────────────────────────
local function handle_manual_key(key)
    local step = MANUAL_STEP
    if     key == keys.w then manual_fwd  = clamp(manual_fwd  + step, -1, 1)
    elseif key == keys.s then manual_fwd  = clamp(manual_fwd  - step, -1, 1)
    elseif key == keys.a then manual_side = clamp(manual_side - step, -1, 1)
    elseif key == keys.d then manual_side = clamp(manual_side + step, -1, 1)
    elseif key == keys.r then manual_up   = clamp(manual_up   + step, -1, 1)
    elseif key == keys.f then manual_up   = clamp(manual_up   - step, -1, 1)
    elseif key == keys.q then manual_yaw  = clamp(manual_yaw  - step, -1, 1)
    elseif key == keys.e then manual_yaw  = clamp(manual_yaw  + step, -1, 1)
    elseif key == keys.x then
        manual_fwd=0; manual_side=0; manual_up=0; manual_yaw=0
        if nav then nav.helm.setThrottle(0, 0, 0) end
        gui:log("Manual: all stop", "WARN")
        return
    elseif key == keys.m then
        MODE = "AUTO"
        manual_fwd=0; manual_side=0; manual_up=0; manual_yaw=0
        if nav then nav.helm.setThrottle(0, 0, 0) end
        gui:log("Switched to AUTO mode", "OK")
        gui.input_prompt = "> "
        return
    else return end
    apply_manual()
    gui:log(string.format(
        "Manual fwd:%.2f side:%.2f up:%.2f",
        manual_fwd, manual_side, manual_up), "INFO")
end

-- ── Command handler ───────────────────────────────────────────
local function handle(cmd_str)
    local parts = {}
    for w in cmd_str:gmatch("%S+") do table.insert(parts, w) end
    if #parts == 0 then return end
    local cmd = parts[1]:lower()

    -- scan works without helm
    if cmd == "scan" then
        local names = peripheral.getNames()
        if #names == 0 then
            gui:log("No peripherals found", "WARN")
        else
            for _, n in ipairs(names) do
                local t = peripheral.getType(n)
                gui:log("  " .. n .. " -> " .. tostring(t), "INFO")
            end
        end
        return
    end

    if cmd == "manual" then
        if not nav then gui:log("No helm for manual control","ERR"); return end
        if nav then nav:stop() end
        MODE = "MANUAL"
        manual_fwd=0; manual_side=0; manual_up=0; manual_yaw=0
        gui.input_prompt = "[M]> "
        gui:log("MANUAL mode: W/S=fwd  A/D=strafe  R/F=up/dn  Q/E=yaw  X=stop  M=exit", "WARN")
        gui:drawInput()
        return
    end

    if not nav then gui:log("No helm, cmd ignored","WARN"); return end

    if cmd == "goto" then
        local x,y,z = tonumber(parts[2]),tonumber(parts[3]),tonumber(parts[4])
        if not (x and y and z) then gui:log("Usage: goto <x> <y> <z>","WARN"); return end
        nav:clearWaypoints(); nav:addWaypoint(x,y,z); nav:start()
        MODE = "AUTO"
        gui:log(string.format("Target set (%.0f, %.0f, %.0f)", x, y, z), "OK")

    elseif cmd == "wp" then
        local sub = (parts[2] or ""):lower()
        if sub == "add" then
            local x,y,z = tonumber(parts[3]),tonumber(parts[4]),tonumber(parts[5])
            if not (x and y and z) then gui:log("Usage: wp add <x> <y> <z>","WARN"); return end
            nav:addWaypoint(x,y,z)
            gui:log(string.format("WP #%d added (%.0f, %.0f, %.0f)", #nav.waypoints, x, y, z), "OK")
        elseif sub == "clear" then
            nav:clearWaypoints(); gui:log("All waypoints cleared","WARN")
        else gui:log("wp sub-cmd: add | clear","INFO") end

    elseif cmd == "start" then
        MODE = "AUTO"; nav:start(); gui:log("Navigation started","OK")
    elseif cmd == "stop" then
        nav:stop(); gui:log("Stopped","WARN")

    elseif cmd == "pos" then
        local s = nav:getStatus()
        gui:log(string.format("Pos: X=%.1f Y=%.1f Z=%.1f Yaw=%.1f",
            s.pos.x, s.pos.y, s.pos.z, s.yaw), "INFO")

    elseif cmd == "tune" then
        local axis = (parts[2] or ""):lower()
        local kp,ki,kd = tonumber(parts[3]),tonumber(parts[4]),tonumber(parts[5])
        if not (kp and ki and kd) then gui:log("Usage: tune h|v <kp> <ki> <kd>","WARN"); return end
        if axis == "h" then
            nav.pid3.x:tune(kp,ki,kd); nav.pid3.z:tune(kp,ki,kd)
            gui:log(string.format("Horiz PID kp=%.3f ki=%.4f kd=%.3f",kp,ki,kd),"OK")
        elseif axis == "v" then
            nav.pid3.y:tune(kp,ki,kd)
            gui:log(string.format("Vert PID kp=%.3f ki=%.4f kd=%.3f",kp,ki,kd),"OK")
        else gui:log("Axis: h=horizontal  v=vertical","INFO") end

    elseif cmd == "help" then
        gui:log("--- Commands ---","INFO")
        gui:log("scan          list attached peripherals","INFO")
        gui:log("goto x y z    fly to coordinate","INFO")
        gui:log("wp add x y z  add waypoint","INFO")
        gui:log("wp clear      clear all waypoints","INFO")
        gui:log("start/stop    begin/halt navigation","INFO")
        gui:log("manual        enter manual keyboard control","INFO")
        gui:log("pos           print current position","INFO")
        gui:log("tune h|v ...  adjust PID gains","INFO")

    elseif cmd == "exit" or cmd == "quit" then
        if nav then nav:stop() end
        gui:log("Exiting...","WARN"); os.sleep(0.3)
        term.clear(); term.setCursorPos(1,1)
        error("__EXIT__")
    else
        gui:log("Unknown: "..cmd.."  (help for list)","WARN")
    end
end

-- ── Button actions ────────────────────────────────────────────
local function on_button(action)
    if action == "goto" then
        local r = gui:dialog("Goto Coord", {
            {label="X:", default="0"},
            {label="Y:", default="80"},
            {label="Z:", default="0"},
        })
        if r then handle(string.format("goto %s %s %s", r[1], r[2], r[3])) end

    elseif action == "wp_add" then
        local r = gui:dialog("Add Waypoint", {
            {label="X:", default="0"},
            {label="Y:", default="80"},
            {label="Z:", default="0"},
        })
        if r then handle(string.format("wp add %s %s %s", r[1], r[2], r[3])) end

    elseif action == "wp_clear" then handle("wp clear")
    elseif action == "wp_list"  then
        if nav then
            gui:log(string.format("Total %d waypoints", #nav.waypoints), "INFO")
        end
    elseif action == "start"    then handle("start")
    elseif action == "stop"     then handle("stop")
    elseif action == "help"     then handle("help")
    end
    gui:drawButtons(nil)
end

-- ── Main loops ────────────────────────────────────────────────
local dt           = Config.TICK_RATE
local RENDER_EVERY = 4
local tick_count   = 0
local last_nav_t   = os.clock()

local function nav_loop()
    while true do
        local now     = os.clock()
        local elapsed = now - last_nav_t
        last_nav_t    = now

        if nav and MODE == "AUTO" then
            nav:update(elapsed > 0 and elapsed or dt)
        end

        tick_count = tick_count + 1
        -- Skip render while a dialog modal is open
        if tick_count % RENDER_EVERY == 0 and not gui.modal then
            local s        = safe_status()
            local wps, wi  = safe_waypoints()
            gui:render(s, wps, wi)
        end

        os.sleep(dt)
    end
end

local function event_loop()
    while true do
        local ev, p1, p2, p3 = os.pullEvent()

        -- In MANUAL mode, letter keys control thrust
        if MODE == "MANUAL" and ev == "key" then
            handle_manual_key(p1)
            -- don't process as text input
        elseif ev == "mouse_click" then
            if not gui.modal then
                local btn_action = gui:hitButton(p2, p3)
                if btn_action then
                    gui:drawButtons(btn_action)
                    on_button(btn_action)
                end
            end

        elseif ev == "char" and MODE == "AUTO" then
            gui.input_buf = gui.input_buf .. p1
            gui:drawInput()

        elseif ev == "key" and MODE == "AUTO" then
            if p1 == keys.enter then
                local cmd = gui.input_buf:match("^%s*(.-)%s*$")
                gui.input_buf = ""
                if cmd ~= "" then
                    gui:log("> " .. cmd, "INFO")
                    local ok, err = pcall(handle, cmd)
                    if not ok then
                        if tostring(err):find("__EXIT__") then return end
                        gui:log("Error: " .. tostring(err), "ERR")
                    end
                end
                gui:drawInput()
            elseif p1 == keys.backspace then
                if #gui.input_buf > 0 then
                    gui.input_buf = gui.input_buf:sub(1, -2)
                    gui:drawInput()
                end
            elseif p1 == keys.delete then
                gui.input_buf = ""
                gui:drawInput()
            end

        elseif ev == "term_resize" then
            gui.W, gui.H = term.getSize()
            gui:drawFrame()
            gui:drawLog()
        end
    end
end

local ok2, err2 = pcall(function()
    parallel.waitForAny(nav_loop, event_loop)
end)

if nav then pcall(function() nav:stop() end) end
term.clear()
term.setCursorPos(1, 1)
term.setTextColor(colors.white)
term.setBackgroundColor(colors.black)
if not ok2 and not tostring(err2):find("__EXIT__") then
    print("Crashed: " .. tostring(err2))
else
    print("Autopilot exited. Thrust cleared.")
end
