-- =============================================================
--  quad/check.lua
--  Run this first to diagnose the environment before main.lua
--  Usage: dofile("/quad/check.lua")
-- =============================================================
term.setBackgroundColor(colors.black)
term.setTextColor(colors.white)
term.clear()
term.setCursorPos(1,1)

local W, H = term.getSize()
print("=== Quad FC Environment Check ===")
print("Screen: " .. W .. "x" .. H)
print("Advanced: " .. tostring(term.isColor and term.isColor() or false))
print("")

-- check files
local files = {
    "/quad/config.lua",
    "/quad/imu.lua",
    "/quad/mixer.lua",
    "/quad/controller.lua",
    "/quad/gui.lua",
    "/quad/main.lua",
    "/autopilot/pid.lua",
}
print("-- Files --")
for _, f in ipairs(files) do
    local ok = fs.exists(f)
    print((ok and "[OK] " or "[!!] ") .. f)
end
print("")

-- check peripherals
print("-- Peripherals --")
local names = peripheral.getNames()
for _, n in ipairs(names) do
    print("  " .. n .. " = " .. peripheral.getType(n))
end
if #names == 0 then print("  (none)") end
print("")

-- check sensors + try reading actual values
local sensor_types = {"gimbal_sensor","altitude_sensor","velocity_sensor","Create_RotationSpeedController"}
print("-- Key sensors --")
for _, t in ipairs(sensor_types) do
    local p = peripheral.find(t)
    if p then
        local val = ""
        if t == "gimbal_sensor" then
            local ok, v = pcall(function() return p.getAngles() end)
            if ok and type(v) == "table" then
                val = string.format(" => pitch=%.1f roll=%.1f yaw=%.1f", v[1] or 0, v[2] or 0, v[3] or 0)
            else
                val = " => READ FAILED: " .. tostring(v)
            end
        elseif t == "altitude_sensor" then
            local ok, v = pcall(function() return p.getHeight() end)
            if ok then val = " => height=" .. tostring(v)
            else val = " => READ FAILED: " .. tostring(v) end
        elseif t == "velocity_sensor" then
            local ok, v = pcall(function() return p.getVelocity() end)
            if ok then val = " => vel=" .. tostring(v)
            else val = " => READ FAILED: " .. tostring(v) end
        end
        print("[OK] " .. t .. val)
    else
        print("[--] " .. t)
    end
end
print("")
print("If all files OK and Advanced=true, run:")
print("  /quad/main")
