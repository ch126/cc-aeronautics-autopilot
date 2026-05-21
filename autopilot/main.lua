-- =============================================================
--  autopilot/main.lua  (图形化版本)
--  事件驱动主循环：导航更新 + 鼠标/键盘 + GUI 渲染
--
--  操作方式：
--    鼠标点击按钮  ──  触发对应操作
--    底部输入框    ──  键盘输入命令（同旧版命令集）
--    Ctrl+T        ──  退出
-- =============================================================

package.path = package.path .. ";/autopilot/?.lua;/?.lua"

local Nav    = require("autopilot.nav")
local GUI    = require("autopilot.gui")
local Config = require("autopilot.config")

-- ── 初始化 ────────────────────────────────────────────────────
local gui = GUI.new()
gui:drawFrame()
gui:log("系统初始化中...", "INFO")

local ok_nav, nav = pcall(Nav.new)
if not ok_nav then
    gui:log("飞艇控制器未找到: " .. tostring(nav), "ERR")
    gui:log("请确认外设连接后重启。", "WARN")
    nav = nil
else
    gui:log("飞艇控制器已连接，系统就绪！", "OK")
end

-- ── 虚拟 nav（无外设时显示占位数据）─────────────────────────
local function safe_status()
    if nav then return nav:getStatus() end
    local Vec3 = require("autopilot.vec3")
    return {
        state="ERROR", msg="飞艇控制器未连接",
        pos=Vec3.new(0,0,0), velocity=Vec3.new(0,0,0), yaw=0,
        wp_current=0, wp_total=0, target=nil, dist=0,
        total_dist=0, elapsed=0,
    }
end

local function safe_waypoints()
    if nav then return nav.waypoints, nav.wp_index end
    return {}, 1
end

-- ── 命令处理器 ────────────────────────────────────────────────
local function handle(cmd_str)
    if not nav then gui:log("无飞艇连接，命令忽略", "WARN"); return end
    local parts = {}
    for w in cmd_str:gmatch("%S+") do table.insert(parts, w) end
    if #parts == 0 then return end
    local cmd = parts[1]:lower()

    if cmd == "goto" then
        local x,y,z = tonumber(parts[2]),tonumber(parts[3]),tonumber(parts[4])
        if not (x and y and z) then gui:log("用法: goto <x> <y> <z>","WARN"); return end
        nav:clearWaypoints(); nav:addWaypoint(x,y,z); nav:start()
        gui:log(string.format("目标设定 (%.0f,%.0f,%.0f)",x,y,z),"OK")

    elseif cmd == "wp" then
        local sub = (parts[2] or ""):lower()
        if sub == "add" then
            local x,y,z = tonumber(parts[3]),tonumber(parts[4]),tonumber(parts[5])
            if not (x and y and z) then gui:log("用法: wp add <x> <y> <z>","WARN"); return end
            nav:addWaypoint(x,y,z)
            gui:log(string.format("航点 #%d 已添加 (%.0f,%.0f,%.0f)",#nav.waypoints,x,y,z),"OK")
        elseif sub == "clear" then
            nav:clearWaypoints(); gui:log("所有航点已清空","WARN")
        else gui:log("wp 子命令: add | clear","INFO") end

    elseif cmd == "start" then nav:start(); gui:log("导航已启动","OK")
    elseif cmd == "stop"  then nav:stop();  gui:log("飞艇已停止","WARN")

    elseif cmd == "tune" then
        local axis = (parts[2] or ""):lower()
        local kp,ki,kd = tonumber(parts[3]),tonumber(parts[4]),tonumber(parts[5])
        if not (kp and ki and kd) then gui:log("用法: tune h|v <kp> <ki> <kd>","WARN"); return end
        if axis == "h" then
            nav.pid3.x:tune(kp,ki,kd); nav.pid3.z:tune(kp,ki,kd)
            gui:log(string.format("水平PID kp=%.3f ki=%.4f kd=%.3f",kp,ki,kd),"OK")
        elseif axis == "v" then
            nav.pid3.y:tune(kp,ki,kd)
            gui:log(string.format("垂直PID kp=%.3f ki=%.4f kd=%.3f",kp,ki,kd),"OK")
        else gui:log("轴参数: h=水平  v=垂直","INFO") end

    elseif cmd == "help" then
        gui:log("goto / wp add / wp clear / start / stop / tune h|v","INFO")
    elseif cmd == "exit" or cmd == "quit" then
        if nav then nav:stop() end
        gui:log("正在退出...","WARN"); os.sleep(0.3)
        term.clear(); term.setCursorPos(1,1)
        error("__EXIT__")
    else
        gui:log("未知命令: "..cmd.."  (输入 help)","WARN")
    end
end

-- ── 按钮动作 ──────────────────────────────────────────────────
local function on_button(action)
    if action == "goto" then
        local r = gui:dialog("前往坐标",{
            {label="X 坐标:", default="0"},
            {label="Y 坐标:", default="80"},
            {label="Z 坐标:", default="0"},
        })
        if r then handle(string.format("goto %s %s %s",r[1],r[2],r[3])) end

    elseif action == "wp_add" then
        local r = gui:dialog("添加航点",{
            {label="X 坐标:", default="0"},
            {label="Y 坐标:", default="80"},
            {label="Z 坐标:", default="0"},
        })
        if r then handle(string.format("wp add %s %s %s",r[1],r[2],r[3])) end

    elseif action == "wp_clear" then handle("wp clear")
    elseif action == "wp_list"  then
        if nav then gui:log(string.format("共 %d 个航点（见右侧面板）",#nav.waypoints),"INFO") end
    elseif action == "start"    then handle("start")
    elseif action == "stop"     then handle("stop")
    elseif action == "help"     then handle("help")
    end
    gui:drawButtons(nil)
end

-- ── 主循环 ────────────────────────────────────────────────────
local dt           = Config.TICK_RATE
local RENDER_EVERY = 4
local tick_count   = 0
local last_nav_t   = os.clock()

local function nav_loop()
    while true do
        local now = os.clock()
        local elapsed = now - last_nav_t
        last_nav_t = now
        if nav then nav:update(elapsed > 0 and elapsed or dt) end
        tick_count = tick_count + 1
        if tick_count % RENDER_EVERY == 0 then
            local s = safe_status()
            local wps,wi = safe_waypoints()
            gui:render(s, wps, wi)
        end
        os.sleep(dt)
    end
end

local function event_loop()
    while true do
        local ev,p1,p2,p3 = os.pullEvent()

        if ev == "mouse_click" then
            local btn_action = gui:hitButton(p2, p3)
            if btn_action then
                gui:drawButtons(btn_action)
                on_button(btn_action)
            end

        elseif ev == "char" then
            gui.input_buf = gui.input_buf .. p1
            gui:drawInput()

        elseif ev == "key" then
            if p1 == keys.enter then
                local cmd = gui.input_buf:match("^%s*(.-)%s*$")
                gui.input_buf = ""
                if cmd ~= "" then
                    gui:log("> "..cmd, "INFO")
                    local ok,err = pcall(handle, cmd)
                    if not ok then
                        if tostring(err):find("__EXIT__") then return end
                        gui:log("错误: "..tostring(err), "ERR")
                    end
                end
                gui:drawInput()
            elseif p1 == keys.backspace then
                if #gui.input_buf > 0 then
                    gui.input_buf = gui.input_buf:sub(1,-2)
                    gui:drawInput()
                end
            elseif p1 == keys.delete then
                gui.input_buf = ""; gui:drawInput()
            end

        elseif ev == "term_resize" then
            gui.W, gui.H = term.getSize()
            gui:drawFrame(); gui:drawLog()
        end
    end
end

local ok2, err2 = pcall(function()
    parallel.waitForAny(nav_loop, event_loop)
end)

if nav then pcall(function() nav:stop() end) end
term.clear(); term.setCursorPos(1,1)
term.setTextColor(colors.white); term.setBackgroundColor(colors.black)
if not ok2 and not tostring(err2):find("__EXIT__") then
    print("程序异常退出: " .. tostring(err2))
else
    print("自动驾驶已退出，推力已清零。")
end
