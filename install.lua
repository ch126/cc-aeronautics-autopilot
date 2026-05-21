-- =============================================================
--  install.lua
--  一键部署脚本 —— 在 CC:Tweaked 电脑中运行
--
--  【推荐】游戏内一键安装（仅需运行这一条命令）：
--
--    wget https://raw.githubusercontent.com/ch126/cc-aeronautics-autopilot/main/install.lua
--    lua install.lua
--
--  安装完成后直接运行：
--    lua autopilot/main.lua
--  或重启电脑（startup.lua 会自动启动）
-- =============================================================

local BASE_URL = "https://raw.githubusercontent.com/ch126/cc-aeronautics-autopilot/main/"

-- 要下载的文件列表（相对路径）
local FILES = {
    "autopilot/config.lua",
    "autopilot/vec3.lua",
    "autopilot/pid.lua",
    "autopilot/obstacle.lua",
    "autopilot/nav.lua",
    "autopilot/gui.lua",
    "autopilot/main.lua",
    "startup.lua",
}

local function mkdir(path)
    -- 确保目录存在
    if not fs.exists(path) then
        fs.makeDir(path)
    end
end

local function download(url, dest)
    local resp = http.get(url)
    if not resp then
        printError("下载失败: " .. url)
        return false
    end
    local content = resp.readAll()
    resp.close()

    -- 确保父目录存在
    local dir = fs.getDir(dest)
    if dir ~= "" then mkdir(dir) end

    local f = fs.open(dest, "w")
    f.write(content)
    f.close()
    print("  ✓ " .. dest)
    return true
end

print("====================================")
print("  Create:Aeronautics 自动驾驶安装器")
print("====================================")
mkdir("autopilot")

local success = 0
for _, file in ipairs(FILES) do
    local url = BASE_URL .. file
    if download(url, "/" .. file) then
        success = success + 1
    end
end

print(string.format("\n安装完成 %d/%d 个文件", success, #FILES))
if success == #FILES then
    print("重启后自动运行，或手动执行: lua autopilot/main.lua")
else
    printError("部分文件下载失败，请检查 HTTP 权限和 URL 配置")
end
