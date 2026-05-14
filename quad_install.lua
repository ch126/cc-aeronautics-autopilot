-- =============================================================
--  quad_install.lua
--  wget https://raw.githubusercontent.com/ch126/cc-aeronautics-autopilot/main/quad_install.lua
--  lua quad_install.lua
-- =============================================================

local BASE = "https://raw.githubusercontent.com/ch126/cc-aeronautics-autopilot/main/"

local FILES = {
    "autopilot/pid.lua",
    "quad/config.lua",
    "quad/imu.lua",
    "quad/mixer.lua",
    "quad/controller.lua",
    "quad/gui.lua",
    "quad/main.lua",
    "quad/check.lua",
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
print("  Quad FC Installer")
print("======================================")
mkdir("autopilot"); mkdir("quad")

local ok = 0
for _, f in ipairs(FILES) do
    if download(BASE .. f, "/" .. f) then ok = ok + 1 end
end

print(string.format("\nInstalled %d/%d files", ok, #FILES))

if ok == #FILES then
    -- write startup.lua (same style as original autopilot startup.lua)
    local s = fs.open("/startup.lua", "w")
    s.write([[
_LOADED = {}
local _real_dofile = dofile
function dofile(path)
    if _LOADED[path] then return _LOADED[path] end
    local result = _real_dofile(path)
    _LOADED[path] = result or true
    return result
end

os.sleep(1)
term.clear()
term.setCursorPos(1, 1)
term.setTextColor(colors.cyan)
print("+--------------------------------------------+")
print("|   Quad Flight Controller  Booting...       |")
print("+--------------------------------------------+")
term.setTextColor(colors.white)

shell.setDir("/")
dofile("/quad/main.lua")
]])
    s.close()
    print("startup.lua written.")
    print("")
    print("Edit /quad/config.lua to set motor names.")
    print("Then reboot: os.reboot()")
else
    print("ERROR: some files failed to download.")
end
