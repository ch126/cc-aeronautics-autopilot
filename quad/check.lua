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

-- check sensors
local sensor_types = {"gimbal_sensor","altitude_sensor","velocity_sensor","Create_SpeedController"}
print("-- Key sensors --")
for _, t in ipairs(sensor_types) do
    local p = peripheral.find(t)
    print((p and "[OK] " or "[--] ") .. t)
end
print("")
print("If all files OK and Advanced=true, run:")
print("  dofile('/quad/main.lua')")
