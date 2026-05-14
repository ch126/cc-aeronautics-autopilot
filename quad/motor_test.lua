-- =============================================================
--  quad/motor_test.lua
--  Direct speed controller test - run this to diagnose motor issues
--  Usage: /quad/motor_test
-- =============================================================
term.clear()
term.setCursorPos(1,1)
print("=== Motor Test ===")
print("")

-- 1. list all peripherals
print("All peripherals:")
local names = peripheral.getNames()
if #names == 0 then
    print("  (none connected!)")
    print("  >> Check wired modem cables are connected and active")
    return
end
for _, n in ipairs(names) do
    print("  " .. n .. "  type=" .. peripheral.getType(n))
end
print("")

-- 2. find speed controllers
print("Finding Create_RotationSpeedController ...")
local gears = { peripheral.find("Create_RotationSpeedController") }
print("Found: " .. #gears)
if #gears == 0 then
    print("  >> NOT FOUND. Check peripheral type name.")
    print("  >> Available types above - look for 'speed' or 'rotation'")
    return
end
print("")

-- 3. list methods on first gear
local g = gears[1]
print("Methods on gear[1]:")
for k, v in pairs(g) do
    print("  " .. tostring(k) .. " = " .. type(v))
end
print("")

-- 4. try setTargetSpeed
print("Testing setTargetSpeed(64) ...")
local ok, err = pcall(function()
    g.setTargetSpeed(64)
end)
if ok then
    print("  >> SUCCESS! Motor should be spinning at 64 RPM")
else
    print("  >> FAILED: " .. tostring(err))
end

print("")
print("Press any key to set speed back to 0 ...")
os.pullEvent("key")
g.setTargetSpeed(0)
print("Speed set to 0. Done.")
