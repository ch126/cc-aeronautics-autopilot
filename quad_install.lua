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

print("=== 四旋翼飞控安装程序 ===")

for _, f in ipairs(FILES) do
    -- 创建父目录
    local dir = f.dst:match("^(.*)/[^/]+$")
    if dir and dir ~= "" then
        fs.makeDir(dir)
    end

    io.write("下载 " .. f.dst .. " ... ")
    local ok, err = pcall(function()
        local h = http.get(f.src)
        if not h then error("HTTP 请求失败") end
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
print("安装完成！")
print("")
print("请编辑 /quad/config.lua 填写：")
print("  1. C.MOTOR_FL / FR / BR / BL  -- 转速控制器外设名称")
print("  2. 如果电机名称与 'Create_SpeedController' 不同，修改 C.MOTOR_TYPE")
print("  3. 调整 RPM_HOVER / RPM_MAX 以匹配你的螺旋桨")
print("")
print("启动命令：dofile('/quad/main.lua')")
print("或将以下内容加入 startup.lua：")
print("  dofile('/quad/main.lua')")
