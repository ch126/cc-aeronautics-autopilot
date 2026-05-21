-- =============================================================
  -- autopilot/main.lua  ()
  -- + / + GUI
--



  -- Ctrl+T
-- =============================================================

package.path = package.path .. ";/autopilot/?.lua;/?.lua"

local Nav    = require("autopilot.nav")
local GUI    = require("autopilot.gui")
local Config = require("autopilot.config")


local gui = GUI.new()
gui:drawFrame()
gui:log("Initializing system...", "INFO")

local ok_nav, nav = pcall(Nav.new)
if not ok_nav then
    gui:log("Helm not found: " .. tostring(nav), "ERR")
    gui:log("Check peripheral connection and reboot.", "WARN")
    nav = nil
else
    gui:log("Helm connected. System ready!", "OK")
end

  -- nav
local function safe_status()
    if nav then return nav:getStatus() end
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


local function handle(cmd_str)
    if not nav then gui:log("No helm connected, cmd ignored", "WARN"); return end
    local parts = {}
    for w in cmd_str:gmatch("%S+") do table.insert(parts, w) end
    if #parts == 0 then return end
    local cmd = parts[1]:lower()

    if cmd == "goto" then
        local x,y,z = tonumber(parts[2]),tonumber(parts[3]),tonumber(parts[4])
        if not (x and y and z) then gui:log("Usage: goto <x> <y> <z>","WARN"); return end
        nav:clearWaypoints(); nav:addWaypoint(x,y,z); nav:start()
        gui:log(string.format("Target set (%.0f,%.0f,%.0f)",x,y,z),"OK")

    elseif cmd == "wp" then
        local sub = (parts[2] or ""):lower()
        if sub == "add" then
            local x,y,z = tonumber(parts[3]),tonumber(parts[4]),tonumber(parts[5])
            if not (x and y and z) then gui:log("Usage: wp add <x> <y> <z>","WARN"); return end
            nav:addWaypoint(x,y,z)
            gui:log(string.format("WP #%d added (%.0f,%.0f,%.0f)",#nav.waypoints,x,y,z),"OK")
        elseif sub == "clear" then
            nav:clearWaypoints(); gui:log("All waypoints cleared","WARN")
        else gui:log("wp sub-cmd: add | clear","INFO") end

    elseif cmd == "start" then nav:start(); gui:log("Navigation started","OK")
    elseif cmd == "stop"  then nav:stop();  gui:log("Airship stopped","WARN")

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
        gui:log("goto/wp add/wp clear/start/stop/tune h|v","INFO")
    elseif cmd == "exit" or cmd == "quit" then
        if nav then nav:stop() end
        gui:log("Exiting...","WARN"); os.sleep(0.3)
        term.clear(); term.setCursorPos(1,1)
        error("__EXIT__")
    else
        gui:log("Unknown cmd: "..cmd.."  (type help)","WARN")
    end
end


local function on_button(action)
    if action == "goto" then
        local r = gui:dialog("Goto Coord",{
            {label="X Coord:", default="0"},
            {label="Y Coord:", default="80"},
            {label="Z Coord:", default="0"},
        })
        if r then handle(string.format("goto %s %s %s",r[1],r[2],r[3])) end

    elseif action == "wp_add" then
        local r = gui:dialog("Add Waypoint",{
            {label="X Coord:", default="0"},
            {label="Y Coord:", default="80"},
            {label="Z Coord:", default="0"},
        })
        if r then handle(string.format("wp add %s %s %s",r[1],r[2],r[3])) end

    elseif action == "wp_clear" then handle("wp clear")
    elseif action == "wp_list"  then
        if nav then gui:log(string.format("Total %d WPs (see right panel)",#nav.waypoints),"INFO") end
    elseif action == "start"    then handle("start")
    elseif action == "stop"     then handle("stop")
    elseif action == "help"     then handle("help")
    end
    gui:drawButtons(nil)
end


local dt           = Config.TICK_RATE
local RENDER_EVERY = 4
local tick_count   = 0
local last_nav_t   = os.clock()

local function nav_loop()
    while true do
        local now = os.clock()
        local elapsed = now - last_nav_t
        last_nav_t = now
        if nav then nav:update(elapsed > 0 and elapsed or dt) end
        tick_count = tick_count + 1
        if tick_count % RENDER_EVERY == 0 then
            local s = safe_status()
            local wps,wi = safe_waypoints()
            gui:render(s, wps, wi)
        end
        os.sleep(dt)
    end
end

local function event_loop()
    while true do
        local ev,p1,p2,p3 = os.pullEvent()

        if ev == "mouse_click" then
            local btn_action = gui:hitButton(p2, p3)
            if btn_action then
                gui:drawButtons(btn_action)
                on_button(btn_action)
            end

        elseif ev == "char" then
            gui.input_buf = gui.input_buf .. p1
            gui:drawInput()

        elseif ev == "key" then
            if p1 == keys.enter then
                local cmd = gui.input_buf:match("^%s*(.-)%s*$")
                gui.input_buf = ""
                if cmd ~= "" then
                    gui:log("> "..cmd, "INFO")
                    local ok,err = pcall(handle, cmd)
                    if not ok then
                        if tostring(err):find("__EXIT__") then return end
                        gui:log("Error: "..tostring(err), "ERR")
                    end
                end
                gui:drawInput()
            elseif p1 == keys.backspace then
                if #gui.input_buf > 0 then
                    gui.input_buf = gui.input_buf:sub(1,-2)
                    gui:drawInput()
                end
            elseif p1 == keys.delete then
                gui.input_buf = ""; gui:drawInput()
            end

        elseif ev == "term_resize" then
            gui.W, gui.H = term.getSize()
            gui:drawFrame(); gui:drawLog()
        end
    end
end

local ok2, err2 = pcall(function()
    parallel.waitForAny(nav_loop, event_loop)
end)

if nav then pcall(function() nav:stop() end) end
term.clear(); term.setCursorPos(1,1)
term.setTextColor(colors.white); term.setBackgroundColor(colors.black)
if not ok2 and not tostring(err2):find("__EXIT__") then
    print("Crashed: " .. tostring(err2))
else
    print("Autopilot exited. Thrust cleared.")
end
