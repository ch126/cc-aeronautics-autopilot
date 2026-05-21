-- =============================================================
--  startup.lua
--  开机自启 —— 放置于电脑根目录 /startup.lua
--  CC:Tweaked 会在开机时自动执行此文件
-- =============================================================

-- 等待 1 秒让外设初始化完成
os.sleep(1)

-- 清屏并打印 Logo
term.clear()
term.setCursorPos(1, 1)
term.setTextColor(colors.cyan)
print("╔══════════════════════════════════════════════╗")
print("║   Create:Aeronautics 自动驾驶系统 启动中...  ║")
print("╚══════════════════════════════════════════════╝")
term.setTextColor(colors.white)

-- 运行主程序
shell.run("autopilot/main.lua")
