-- =============================================================
--  autopilot/main.lua  (Create:Aeronautics Simulated Project)
--
--  Commands:
--    scan              list all peripherals
--    sensors           show sensor connection status
--    hover             hold current altitude, stop moving
--    alt <Y>           set altitude target (hold at Y blocks)
--    hdg <deg>         set heading target (0=north, 90=east)
--    speed <v>         set target forward speed (m/s)
--    fly <alt> <hdg> <spd>   fly at altitude/heading/speed
--    goto <dX> <dZ> [alt]    fly to relative offset (dead reckoning)
--    stop              stop all outputs
--    pos               show dead-reckoning position
--    pos reset         reset dead-reckoning origin
--    manual            enter manual redstone control
--    tune alt|spd|hdg <kp> <ki> <kd>   adjust PID gains
--    help              show this list
--    exit / quit       shutdown
--
--  MANUAL mode keys:
--    W/S = forward/back   A/D = yaw left/right
--    R/F = up/down        X = stop   M = exit manual
-- =============================================================

-- 绝对路径，不依赖 CWD，在 CC:Tweaked 中 require("autopilot.gui") 
-- 会将 "." 替换为 "/" 得到 "autopilot/gui"，配合 "/?.lua" 得到 "/autopilot/gui.lua"
package.path = "/?.lua"

local Nav    = require("autopilot.nav")
local GUI    = require("autopilot.gui")
local Config = require("autopilot.config")

-- ── Init ──────────────────────────────────────────────────────
local gui = GUI.new()
gui:drawFrame()
gui:log("Initializing...", "INFO")

local ok_nav, nav = pcall(Nav.new)
if not ok_nav then
    gui:log("Nav init failed: " .. tostring(nav), "ERR")
    gui:log("Continuing without flight control.", "WARN")
    nav = nil
else
    gui:log("Nav ready. " .. nav:getStatus().sensors, "OK")
end

-- ── Mode state ────────────────────────────────────────────────
local MODE        = "AUTO"   -- "AUTO" | "MANUAL"
local MAN_POWER   = 0.7      -- manual thrust level (0-1)

-- ── Timer ─────────────────────────────────────────────────────
local dt           = Config.TICK_RATE
local RENDER_EVERY = 4
local tick_count   = 0
local last_nav_t   = os.clock()
local nav_timer    = os.startTimer(dt)

-- ── Safe status ───────────────────────────────────────────────
local function safe_status()
    if nav then
        local st = nav:getStatus()
        if MODE == "MANUAL" then st.mode = "MANUAL" end
        return st
    end
    return {
        mode="ERROR", msg="Nav not connected",
        altitude=0, speed=0, heading=0, pitch=0, roll=0,
        dr_x=0, dr_z=0, elapsed=0, sensors="--",
        tgt_alt=nil, tgt_spd=0, tgt_hdg=nil,
    }
end

-- ── Manual key handler ────────────────────────────────────────
local function handle_manual_key(key)
    if not nav then return end
    local p = MAN_POWER
    if     key == keys.w then nav:manualSet(p, 0, 0, 0, 0, 0)
    elseif key == keys.s then nav:manualSet(0, p, 0, 0, 0, 0)
    elseif key == keys.r then nav:manualSet(0, 0, p, 0, 0, 0)
    elseif key == keys.f then nav:manualSet(0, 0, 0, p, 0, 0)
    elseif key == keys.a then nav:manualSet(0, 0, 0, 0, p, 0)
    elseif key == keys.d then nav:manualSet(0, 0, 0, 0, 0, p)
    elseif key == keys.x then
        nav:manualSet(0, 0, 0, 0, 0, 0)
        gui:log("Manual: all stop", "WARN")
        return
    elseif key == keys.m then
        nav:manualSet(0, 0, 0, 0, 0, 0)
        MODE = "AUTO"
        gui.input_prompt = "> "
        gui:log("Exited MANUAL mode", "OK")
        gui:drawInput()
        return
    else return end
    gui:log(string.format("Manual: %s", keys.getName(key)), "INFO")
end

-- ── Command handler ───────────────────────────────────────────
local running = true

local function handle(cmd_str)
    local parts = {}
    for w in cmd_str:gmatch("%S+") do table.insert(parts, w) end
    if #parts == 0 then return end
    local cmd = parts[1]:lower()

    -- ── Always available ──────────────────────────────────────
    if cmd == "scan" then
        local names = peripheral.getNames()
        if #names == 0 then
            gui:log("No peripherals found", "WARN")
        else
            for _, n in ipairs(names) do
                gui:log("  " .. n .. " -> " .. tostring(peripheral.getType(n)), "INFO")
            end
        end
        return

    elseif cmd == "sensors" then
        if nav then
            gui:log(nav:getStatus().sensors, "INFO")
        else
            gui:log("Nav not initialized", "WARN")
        end
        return

    elseif cmd == "exit" or cmd == "quit" then
        running = false
        return

    elseif cmd == "help" then
        gui:log("--- Commands ---", "INFO")
        gui:log("scan                   list peripherals", "INFO")
        gui:log("sensors                sensor status", "INFO")
        gui:log("hover                  hold position", "INFO")
        gui:log("alt <Y>                set altitude (blocks)", "INFO")
        gui:log("hdg <deg>              set heading (0=N 90=E)", "INFO")
        gui:log("speed <v>              set forward speed (m/s)", "INFO")
        gui:log("fly <alt> <hdg> <spd>  full fly command", "INFO")
        gui:log("goto <dX> <dZ> [alt]   relative waypoint (DR)", "INFO")
        gui:log("stop                   stop all", "INFO")
        gui:log("pos / pos reset        dead-reckoning position", "INFO")
        gui:log("manual                 keyboard control mode", "INFO")
        gui:log("tune alt|spd|hdg ...   PID tuning", "INFO")
        return
    end

    -- ── Require nav ───────────────────────────────────────────
    if not nav then gui:log("Nav not available", "WARN"); return end

    if cmd == "hover" then
        nav:hover()
        gui:log("Hovering at Y=" .. string.format("%.1f", nav:getStatus().altitude), "OK")

    elseif cmd == "alt" then
        local y = tonumber(parts[2])
        if not y then gui:log("Usage: alt <Y>", "WARN"); return end
        nav:setAltitude(y)
        if nav.mode == "IDLE" then nav.mode = "HOVER" end
        gui:log(string.format("Altitude target: %.1f", y), "OK")

    elseif cmd == "hdg" then
        local h = tonumber(parts[2])
        if not h then gui:log("Usage: hdg <degrees>", "WARN"); return end
        nav:setHeading(h)
        gui:log(string.format("Heading target: %.1f deg", h), "OK")

    elseif cmd == "speed" then
        local v = tonumber(parts[2])
        if not v then gui:log("Usage: speed <m/s>", "WARN"); return end
        nav:setSpeed(v)
        gui:log(string.format("Speed target: %.1f m/s", v), "OK")

    elseif cmd == "fly" then
        local alt = tonumber(parts[2])
        local hdg = tonumber(parts[3])
        local spd = tonumber(parts[4])
        if not (alt and hdg) then
            gui:log("Usage: fly <alt> <hdg> [spd]", "WARN"); return
        end
        nav:fly(alt, hdg, spd)
        gui:log(string.format("Flying alt=%.0f hdg=%.0f spd=%.1f",
            alt, hdg, spd or Config.MAX_SPEED), "OK")

    elseif cmd == "goto" then
        local dx  = tonumber(parts[2])
        local dz  = tonumber(parts[3])
        local alt = tonumber(parts[4])
        if not (dx and dz) then
            gui:log("Usage: goto <dX> <dZ> [alt]", "WARN"); return
        end
        nav:goto_rel(dx, dz, alt)
        gui:log(string.format("Goto dX=%.0f dZ=%.0f (dead reckoning)", dx, dz), "OK")

    elseif cmd == "stop" then
        nav:stop()
        gui:log("Stopped", "WARN")

    elseif cmd == "pos" then
        local sub = (parts[2] or ""):lower()
        if sub == "reset" then
            nav.sensors:resetDR()
            gui:log("Dead-reckoning position reset", "OK")
        else
            local s = nav.sensors
            local st = nav:getStatus()
            gui:log(string.format(
                "DR pos : dX=%.1f  dZ=%.1f  (ODO %.1f blk)",
                s.dr_x, s.dr_z, s.dr_dist), "INFO")
            gui:log(string.format(
                "Flight : alt=%.1f  hdg=%.1f  spd=%.2f m/s",
                st.altitude, st.heading, st.speed), "INFO")
            gui:log(string.format(
                "Velocity: vX=%.2f  vZ=%.2f  (horiz=%.2f  vert=%.2f)",
                s.vx, s.vz, s.horiz_speed, s.vert_speed), "INFO")
            gui:log(string.format(
                "Attitude: pitch=%.1f  roll=%.1f",
                st.pitch, st.roll), "INFO")
        end

    elseif cmd == "manual" then
        nav:stop()
        nav.mode = "MANUAL"
        MODE = "MANUAL"
        gui.input_prompt = "[M]> "
        gui:log("MANUAL: W/S=fwd/rev  A/D=yaw  R/F=up/dn  X=stop  M=exit", "WARN")
        gui:drawInput()

    elseif cmd == "tune" then
        local axis = (parts[2] or ""):lower()
        local kp = tonumber(parts[3])
        local ki = tonumber(parts[4])
        local kd = tonumber(parts[5])
        if not (kp and ki and kd) then
            gui:log("Usage: tune alt|spd|hdg <kp> <ki> <kd>", "WARN"); return
        end
        if axis == "alt" then
            nav.pid_alt:tune(kp, ki, kd)
            gui:log(string.format("ALT PID: kp=%.3f ki=%.4f kd=%.3f", kp, ki, kd), "OK")
        elseif axis == "spd" then
            nav.pid_spd:tune(kp, ki, kd)
            gui:log(string.format("SPD PID: kp=%.3f ki=%.4f kd=%.3f", kp, ki, kd), "OK")
        elseif axis == "hdg" then
            nav.pid_hdg:tune(kp, ki, kd)
            gui:log(string.format("HDG PID: kp=%.3f ki=%.4f kd=%.3f", kp, ki, kd), "OK")
        else
            gui:log("Axis: alt | spd | hdg", "INFO")
        end

    else
        gui:log("Unknown: " .. cmd .. "  (help for list)", "WARN")
    end
end

-- ── Button handler ────────────────────────────────────────────
local function on_button(action)
    if action == "goto" then
        local r = gui:dialog("Goto (Relative, Dead Reckoning)", {
            {label="dX (east+):",  default="0"},
            {label="dZ (south+):", default="0"},
            {label="Alt (Y):",     default=""},
        })
        if r then
            local alt_s = (r[3] ~= "" and r[3]) or nil
            handle(string.format("goto %s %s%s", r[1], r[2],
                alt_s and (" " .. alt_s) or ""))
        end

    elseif action == "wp_add" then
        local r = gui:dialog("Fly To (alt/hdg/spd)", {
            {label="Altitude (Y):", default="80"},
            {label="Heading (deg):", default="0"},
            {label="Speed (m/s):",   default="5"},
        })
        if r then handle(string.format("fly %s %s %s", r[1], r[2], r[3])) end

    elseif action == "wp_clear" then handle("stop")
    elseif action == "wp_list"  then handle("sensors")
    elseif action == "start"    then handle("hover")
    elseif action == "stop"     then handle("stop")
    elseif action == "help"     then handle("help")
    end
    gui:drawButtons(nil)
end

-- ── Nav tick ──────────────────────────────────────────────────
local function nav_tick()
    local now     = os.clock()
    local elapsed = now - last_nav_t
    last_nav_t    = now

    if nav and MODE == "AUTO" then
        nav:update(elapsed > 0 and elapsed or dt)
    end

    tick_count = tick_count + 1
    if tick_count % RENDER_EVERY == 0 then
        local st = safe_status()
        -- Build pseudo waypoints list for status panel
        local wps = {}
        if nav and nav.mode == "GOTO" then
            wps = {{ x = nav.wp_dx, y = nav.target_alt or 0, z = nav.wp_dz }}
        end
        gui:render(st, wps, 1)
    end

    nav_timer = os.startTimer(dt)
end

-- ── Initial render ────────────────────────────────────────────
do
    local st = safe_status()
    gui:render(st, {}, 1)
    gui:drawInput()
end

-- ── Event loop ────────────────────────────────────────────────
while running do
    local ev, p1, p2, p3 = os.pullEvent()

    if ev == "timer" and p1 == nav_timer then
        nav_tick()

    elseif ev == "mouse_click" then
        local btn = gui:hitButton(p2, p3)
        if btn then
            gui:drawButtons(btn)
            on_button(btn)
            nav_timer = os.startTimer(dt)
        end

    elseif MODE == "MANUAL" and ev == "key" then
        handle_manual_key(p1)

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

-- ── Cleanup ───────────────────────────────────────────────────
if nav then pcall(function() nav:stop() end) end
term.clear(); term.setCursorPos(1,1)
term.setTextColor(colors.white)
term.setBackgroundColor(colors.black)
print("Autopilot exited. All redstone outputs cleared.")
