-- =============================================================
--  install.lua
  -- CC:Tweaked
--

--
--    wget https://raw.githubusercontent.com/ch126/cc-aeronautics-autopilot/main/install.lua
--    lua install.lua
--

--    lua autopilot/main.lua
  -- startup.lua
-- =============================================================

local BASE_URL = "https://raw.githubusercontent.com/ch126/cc-aeronautics-autopilot/main/"


local FILES = {
    "autopilot/config.lua",
    "autopilot/pid.lua",
    "autopilot/sensors.lua",
    "autopilot/nav.lua",
    "autopilot/gui.lua",
    "autopilot/main.lua",
    "startup.lua",
}

local function mkdir(path)

    if not fs.exists(path) then
        fs.makeDir(path)
    end
end

local function download(url, dest)
    local resp = http.get(url)
    if not resp then
        print("FAIL: " .. url)
        return false
    end
    local content = resp.readAll()
    resp.close()


    local dir = fs.getDir(dest)
    if dir ~= "" then mkdir(dir) end

    local f = fs.open(dest, "w")
    f.write(content)
    f.close()
    print("  OK: " .. dest)
    return true
end

print("======================================")
print("  Create:Aeronautics Autopilot Installer")
print("======================================")
mkdir("autopilot")

local success = 0
for _, file in ipairs(FILES) do
    local url = BASE_URL .. file
    if download(url, "/" .. file) then
        success = success + 1
    end
end

print(string.format("\nInstalled %d/%d files", success, #FILES))
if success == #FILES then
    print("Reboot or run: lua autopilot/main.lua")
else
    print("ERROR: Some files failed. Check HTTP access and URL.")
end
