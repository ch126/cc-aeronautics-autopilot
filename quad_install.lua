-- =============================================================
--  quad_install.lua
--  在 CC:Tweaked 电脑上运行：
--    pastebin run <此脚本> <GitHub RAW 基础URL>
--  或直接放到电脑上后：  dofile("quad_install.lua")
--
--  用法示例（假设你已将仓库 push 到 GitHub）：
--    BASE = "https://raw.githubusercontent.com/<用户>/<仓库>/main"
-- =============================================================

local BASE = "https://raw.githubusercontent.com/CharlesHuangCharles/auto_drive/main"

local FILES = {
    -- 依赖：PID 控制器（来自 autopilot 模块）
    { src = BASE .. "/autopilot/pid.lua",     dst = "/autopilot/pid.lua"     },
    -- 四旋翼模块
    { src = BASE .. "/quad/config.lua",       dst = "/quad/config.lua"       },
    { src = BASE .. "/quad/imu.lua",          dst = "/quad/imu.lua"          },
    { src = BASE .. "/quad/mixer.lua",        dst = "/quad/mixer.lua"        },
    { src = BASE .. "/quad/controller.lua",   dst = "/quad/controller.lua"   },
    { src = BASE .. "/quad/main.lua",         dst = "/quad/main.lua"         },
}

print("=== Quad Flight Controller Installer ===")

for _, f in ipairs(FILES) do
    local dir = f.dst:match("^(.*)/[^/]+$")
    if dir and dir ~= "" then
        fs.makeDir(dir)
    end

    io.write("Downloading " .. f.dst .. " ... ")
    local ok, err = pcall(function()
        local h = http.get(f.src)
        if not h then error("HTTP request failed") end
        local content = h.readAll()
        h.close()
        local fh = fs.open(f.dst, "w")
        fh.write(content)
        fh.close()
    end)

    if ok then
        print("OK")
    else
        print("FAIL: " .. tostring(err))
    end
end

print("")
print("Install complete!")
print("")
print("Edit /quad/config.lua and set:")
print("  C.MOTOR_FL/FR/BR/BL  -- speed controller peripheral names")
print("  (run peripheral.getNames() in CC console to find them)")
print("  Adjust RPM_HOVER to match your rotors")
print("")
print("Start: dofile('/quad/main.lua')")
print("Or add to startup.lua:")
print("  dofile('/quad/main.lua')")
