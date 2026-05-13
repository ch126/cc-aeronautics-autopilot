-- =============================================================
--  quad/main.lua
--  Quadrotor flight controller main program
--  Parallel: ctrl loop(50Hz) + nav loop(10Hz) + input
-- =============================================================

dofile("/quad/config.lua")  -- 预载以便后续 dofile 缓存命中
local C    = dofile("/quad/config.lua")
local IMU  = dofile("/quad/imu.lua")
local Mix  = dofile("/quad/mixer.lua")
local Ctrl = dofile("/quad/controller.lua")

-- state
local imu_state = {}
local running   = true
local ctrl_dt   = 1 / C.CTRL_HZ
local nav_dt    = 1 / C.NAV_HZ

-- init
local imu   = IMU.new()
local mixer = Mix.new()
local ctrl  = Ctrl.new()

print("[QUAD] Motors: " .. mixer:status())
print("[QUAD] Type 'arm <alt>' to arm, 'help' for commands")

-- ctrl loop 50Hz
local function ctrlLoop()
    local last_t = os.clock()
    while running do
        local now = os.clock()
        local dt  = now - last_t
        last_t    = now

        imu_state = imu:read(dt)

        if ctrl.armed then
            local po, ro, yo = ctrl:updateInner(imu_state, dt)
            mixer:mix(ctrl.throttle_out or C.RPM_HOVER, po, ro, yo)
        else
            mixer:allStop()
        end

        local sleep_t = ctrl_dt - (os.clock() - now)
        if sleep_t > 0.001 then os.sleep(sleep_t) end
    end
end

-- nav loop 10Hz
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

-- help
local function printHelp()
    print("  arm [alt]           arm and take off to altitude (default 5)")
    print("  disarm              disarm (stop all motors)")
    print("  hover               hold current position")
    print("  goto <x> <z> [alt]  fly to coords (needs GPS)")
    print("  alt <h>             set target altitude")
    print("  yaw <deg>           set target yaw")
    print("  pos                 show attitude/position")
    print("  motors              show motor RPM")
    print("  land                land and disarm")
    print("  quit                exit")
end

-- landing sequence
local function doLand()
    print("[QUAD] Landing...")
    ctrl.target_alt = 0.3
    os.sleep(3)
    ctrl:disarm()
    print("[QUAD] Landed and disarmed")
end

-- input loop
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
            print(string.format("[QUAD] Armed, target alt %.1f m", h))

        elseif cmd == "disarm" then
            ctrl:disarm()
            print("[QUAD] Disarmed")

        elseif cmd == "hover" then
            ctrl:hover(imu_state)
            print("[QUAD] Hovering")

        elseif cmd == "goto" then
            if not imu_state.x then
                print("[QUAD] No GPS, cannot use goto")
            else
                local x   = tonumber(parts[2])
                local z   = tonumber(parts[3])
                local alt = tonumber(parts[4]) or ctrl.target_alt
                if x and z then
                    ctrl.target_x   = x
                    ctrl.target_z   = z
                    ctrl.target_alt = alt
                    print(string.format("[QUAD] Goto (%.1f, %.1f) alt %.1f", x, z, alt))
                else
                    print("Usage: goto <x> <z> [alt]")
                end
            end

        elseif cmd == "alt" then
            local h = tonumber(parts[2])
            if h then
                ctrl.target_alt = h
                print(string.format("[QUAD] Target alt -> %.1f m", h))
            end

        elseif cmd == "yaw" then
            local y = tonumber(parts[2])
            if y then
                ctrl.target_yaw = y % 360
                print(string.format("[QUAD] Target yaw -> %.1f deg", ctrl.target_yaw))
            end

        elseif cmd == "pos" then
            local s = imu_state
            if s.pitch then
                print(string.format(
                    "Att: P=%.1f R=%.1f Y=%.1f  Alt=%.2fm  Clmb=%.2fm/s",
                    s.pitch or 0, s.roll or 0, s.yaw or 0,
                    s.alt   or 0, s.climb or 0))
                if s.x then
                    print(string.format("Pos: X=%.2f  Z=%.2f", s.x, s.z))
                end
            else
                print("[QUAD] IMU not ready")
            end

        elseif cmd == "motors" then
            print("[QUAD] " .. mixer:status())

        elseif cmd == "land" then
            doLand()

        elseif cmd == "quit" then
            ctrl:disarm()
            running = false
            print("[QUAD] Quit")

        elseif cmd ~= "" then
            print("Unknown command '" .. cmd .. "', type help")
        end
        end  -- if line
    end
end

-- main
print("[QUAD] Starting, CTRL_HZ=" .. C.CTRL_HZ .. " NAV_HZ=" .. C.NAV_HZ)
parallel.waitForAny(ctrlLoop, navLoop, inputLoop)
mixer:allStop()
print("[QUAD] Stopped")
