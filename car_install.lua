-- =============================================================
--  car_install.lua  —  汽车自动驾驶安装脚本
--
--  在 CC:Tweaked 电脑上运行：
--    wget https://raw.githubusercontent.com/ch126/cc-aeronautics-autopilot/main/car_install.lua
--    lua car_install.lua
-- =============================================================

local BASE = "https://raw.githubusercontent.com/ch126/cc-aeronautics-autopilot/main/"

local FILES = {
    "autopilot/pid.lua",     -- 复用飞控 PID
    "car/config.lua",
    "car/sensors.lua",
    "car/nav.lua",
    "car/main.lua",
}

local function mkdir(p)
    if not fs.exists(p) then fs.makeDir(p) end
end

local function download(url, dest)
    local r = http.get(url)
    if not r then print("FAIL: " .. url); return false end
    local c = r.readAll(); r.close()
    local dir = fs.getDir(dest)
    if dir ~= "" then mkdir(dir) end
    local f = fs.open(dest, "w"); f.write(c); f.close()
    print("  OK: " .. dest)
    return true
end

print("======================================")
print("  Car Autopilot Installer")
print("======================================")
mkdir("autopilot"); mkdir("car")

local ok = 0
for _, f in ipairs(FILES) do
    if download(BASE .. f, "/" .. f) then ok = ok + 1 end
end

print(string.format("\nInstalled %d/%d files", ok, #FILES))
if ok == #FILES then
    print("Run: lua car/main.lua")
else
    print("ERROR: some files failed.")
end
