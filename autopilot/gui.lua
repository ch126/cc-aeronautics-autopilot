-- =============================================================
--  autopilot/gui.lua
--  全图形化 TUI 界面
--
--  布局（51×19 Advanced Computer 标准尺寸）：
--  ┌─────────────────────────────────────────────────────┐
--  │  标题栏                                              │  row 1
--  ├──────────────────────┬──────────────────────────────┤
--  │  飞行状态面板(左)     │  航点列表面板(右)             │  row 2-13
--  ├──────────────────────┴──────────────────────────────┤
--  │  进度条                                              │  row 14
--  ├──────────────────────────────────────────────────────┤
--  │  [GO][WP+][CLR][START][STOP][TUNE][?]  按钮栏       │  row 15-16
--  ├──────────────────────────────────────────────────────┤
--  │  输入框                                              │  row 17
--  ├──────────────────────────────────────────────────────┤
--  │  日志滚动区（2行）                                    │  row 18-19
--  └──────────────────────────────────────────────────────┘
-- =============================================================

local GUI = {}
GUI.__index = GUI

-- ── 颜色主题 ──────────────────────────────────────────────────
local TH = {
    bg          = colors.black,
    title_bg    = colors.blue,
    title_fg    = colors.white,
    panel_bg    = colors.black,
    panel_border= colors.gray,
    panel_title = colors.cyan,
    label       = colors.gray,
    value       = colors.white,
    value_hi    = colors.yellow,
    state_idle  = colors.gray,
    state_nav   = colors.lime,
    state_avoid = colors.orange,
    state_arr   = colors.cyan,
    state_err   = colors.red,
    btn_bg      = colors.gray,
    btn_fg      = colors.white,
    btn_active  = colors.blue,
    btn_stop    = colors.red,
    btn_go      = colors.green,
    progress_bg = colors.gray,
    progress_fg = colors.lime,
    log_bg      = colors.black,
    log_ok      = colors.lime,
    log_warn    = colors.yellow,
    log_err     = colors.red,
    log_info    = colors.lightGray,
    input_bg    = colors.gray,
    input_fg    = colors.white,
    input_cursor= colors.yellow,
    wp_active   = colors.yellow,
    wp_done     = colors.gray,
    wp_pending  = colors.lightGray,
    bar_bg      = colors.gray,
}

-- ── 工具 ──────────────────────────────────────────────────────
local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end

local function write_at(t, x, y, text, fg, bg)
    t.setCursorPos(x, y)
    if fg then t.setTextColor(fg) end
    if bg then t.setBackgroundColor(bg) end
    t.write(text)
end

local function fill_rect(t, x, y, w, h, bg, char)
    char = char or " "
    t.setBackgroundColor(bg)
    local row = string.rep(char, w)
    for dy = 0, h-1 do
        t.setCursorPos(x, y+dy)
        t.write(row)
    end
end

-- 绘制带圆角标题的边框
local function draw_box(t, x, y, w, h, title, border_c, title_c)
    -- 顶边
    write_at(t, x,   y, "┌", border_c, TH.panel_bg)
    write_at(t, x+1, y, string.rep("─", w-2), border_c, TH.panel_bg)
    write_at(t, x+w-1, y, "┐", border_c, TH.panel_bg)
    -- 底边
    write_at(t, x,   y+h-1, "└", border_c, TH.panel_bg)
    write_at(t, x+1, y+h-1, string.rep("─", w-2), border_c, TH.panel_bg)
    write_at(t, x+w-1, y+h-1, "┘", border_c, TH.panel_bg)
    -- 左右边
    for dy = 1, h-2 do
        write_at(t, x,     y+dy, "│", border_c, TH.panel_bg)
        write_at(t, x+w-1, y+dy, "│", border_c, TH.panel_bg)
        -- 内部填充
        write_at(t, x+1, y+dy, string.rep(" ", w-2), TH.value, TH.panel_bg)
    end
    -- 标题
    if title then
        local tlen = #title
        local tx = x + math.floor((w - tlen - 2) / 2)
        write_at(t, tx, y, " "..title.." ", title_c or TH.panel_title, TH.panel_bg)
    end
end

-- 进度条
local function draw_progress(t, x, y, w, pct, label, fg, bg)
    pct = clamp(pct, 0, 1)
    local filled = math.floor(w * pct)
    local empty  = w - filled
    t.setCursorPos(x, y)
    if filled > 0 then
        t.setTextColor(fg or TH.progress_fg)
        t.setBackgroundColor(TH.progress_fg)
        t.write(string.rep(" ", filled))
    end
    if empty > 0 then
        t.setBackgroundColor(TH.progress_bg)
        t.write(string.rep(" ", empty))
    end
    -- 中心文字叠加
    if label then
        local llen = #label
        local lx = x + math.floor((w - llen) / 2)
        if lx >= x and lx + llen <= x + w then
            t.setCursorPos(lx, y)
            t.setTextColor(colors.white)
            t.setBackgroundColor(
                math.floor(w * pct) >= (lx - x + math.ceil(llen/2))
                and TH.progress_fg or TH.progress_bg)
            t.write(label)
        end
    end
end

-- ── 按钮定义 ──────────────────────────────────────────────────
--  每个按钮: { label, x, y, w, color, action_id }
local function make_buttons(W)
    -- 自适应宽度布局（7个按钮均匀分布）
    local btns = {
        { label="  GO  ", color=TH.btn_go,   action="goto"  },
        { label=" WP+  ", color=TH.btn_active,action="wp_add"},
        { label=" CLR  ", color=TH.btn_bg,    action="wp_clear"},
        { label=" LIST ", color=TH.btn_bg,    action="wp_list"},
        { label=" START", color=TH.btn_go,    action="start" },
        { label=" STOP ", color=TH.btn_stop,  action="stop"  },
        { label=" HELP ", color=TH.btn_bg,    action="help"  },
    }
    -- 计算水平分布
    local total_w = 0
    for _, b in ipairs(btns) do total_w = total_w + #b.label + 1 end
    local gap = math.floor((W - total_w) / (#btns + 1))
    local cx = gap + 1
    for _, b in ipairs(btns) do
        b.x = cx
        b.w = #b.label
        cx = cx + b.w + 1 + gap
    end
    return btns
end

-- ── 构造 GUI ──────────────────────────────────────────────────
function GUI.new()
    local W, H = term.getSize()
    local obj = setmetatable({
        W = W, H = H,
        term = term,
        -- 子窗口
        win_status = nil,
        win_wp     = nil,
        win_log    = nil,
        win_input  = nil,
        -- 布局常量（根据屏幕高度自适应）
        ROW_TITLE  = 1,
        ROW_PANEL  = 2,
        ROW_PROG   = nil,
        ROW_BTN    = nil,
        ROW_INPUT  = nil,
        ROW_LOG    = nil,
        -- 按钮
        buttons    = nil,
        -- 输入状态
        input_buf  = "",
        input_cb   = nil,   -- 输入完成回调 function(text)
        input_prompt = "> ",
        -- 日志环形缓冲
        log_lines  = {},
        log_max    = 4,
        -- 内部状态缓存（用于 diff 渲染）
        _last_state = nil,
    }, GUI)

    -- 计算行布局
    -- 面板区域高度 = H - 标题(1) - 进度(1) - 按钮(2) - 输入(1) - 日志(2) = H - 7
    local panel_h = H - 7
    if panel_h < 4 then panel_h = 4 end

    obj.PANEL_H    = panel_h
    obj.ROW_PROG   = 2 + panel_h       -- 进度条行
    obj.ROW_BTN    = obj.ROW_PROG + 1  -- 按钮行
    obj.ROW_INPUT  = obj.ROW_BTN + 2   -- 输入行
    obj.ROW_LOG    = obj.ROW_INPUT + 1  -- 日志起始行
    obj.log_max    = H - obj.ROW_LOG + 1
    if obj.log_max < 1 then obj.log_max = 1 end

    -- 左右面板宽度分割
    local left_w  = math.floor(W * 0.52)
    local right_w = W - left_w

    -- 创建子窗口（左状态面板，右航点面板）
    obj.win_status = window.create(term.current(),
        1, obj.ROW_PANEL, left_w, panel_h, true)
    obj.win_wp = window.create(term.current(),
        left_w + 1, obj.ROW_PANEL, right_w, panel_h, true)

    obj.left_w  = left_w
    obj.right_w = right_w
    obj.buttons = make_buttons(W)

    return obj
end

-- ── 全屏初始化（清屏 + 静态框架）────────────────────────────
function GUI:drawFrame()
    local t = self.term
    local W, H = self.W, self.H

    -- 背景
    fill_rect(t, 1, 1, W, H, TH.bg)

    -- 标题栏
    fill_rect(t, 1, 1, W, 1, TH.title_bg)
    local title = " ✈  Create:Aeronautics PID 自动驾驶  v1.0  |  NeoForge 1.21.1 + CC:Tweaked "
    local tx = math.max(1, math.floor((W - #title) / 2) + 1)
    write_at(t, tx, 1, title, TH.title_fg, TH.title_bg)

    -- 左面板边框
    draw_box(self.win_status, 1, 1, self.left_w, self.PANEL_H,
             "飞行状态", TH.panel_border, TH.panel_title)
    -- 右面板边框
    draw_box(self.win_wp, 1, 1, self.right_w, self.PANEL_H,
             "航点队列", TH.panel_border, TH.panel_title)

    -- 按钮行背景
    fill_rect(t, 1, self.ROW_BTN,   W, 1, TH.bar_bg)
    fill_rect(t, 1, self.ROW_BTN+1, W, 1, TH.bar_bg)
    self:drawButtons()

    -- 输入行
    fill_rect(t, 1, self.ROW_INPUT, W, 1, TH.input_bg)

    -- 日志区背景
    fill_rect(t, 1, self.ROW_LOG, W, self.log_max, TH.log_bg)
    -- 日志区分隔线
    write_at(t, 1, self.ROW_LOG - 1,
        "─── 日志 " .. string.rep("─", W - 9), TH.panel_border, TH.bg)
end

-- ── 绘制按钮 ──────────────────────────────────────────────────
function GUI:drawButtons(highlighted)
    local t = self.term
    local y = self.ROW_BTN
    for _, btn in ipairs(self.buttons) do
        local bg = (btn.action == highlighted) and colors.white or btn.color
        local fg = (btn.action == highlighted) and colors.black or TH.btn_fg
        fill_rect(t, btn.x, y, btn.w, 2, bg)
        write_at(t, btn.x, y,   btn.label, fg, bg)
        -- 第二行加阴影/图标
        local icon = ""
        if btn.action == "goto"    then icon = "  ►    "
        elseif btn.action == "wp_add"  then icon = "  ⊕    "
        elseif btn.action == "wp_clear"then icon = "  ✕    "
        elseif btn.action == "wp_list" then icon = "  ≡    "
        elseif btn.action == "start"   then icon = "  ▶    "
        elseif btn.action == "stop"    then icon = "  ■    "
        elseif btn.action == "help"    then icon = "  ?    "
        end
        icon = icon:sub(1, btn.w)
        while #icon < btn.w do icon = icon .. " " end
        write_at(t, btn.x, y+1, icon, fg, bg)
    end
end

-- ── 状态面板渲染 ──────────────────────────────────────────────
local STATE_COLOR = {
    IDLE      = TH.state_idle,
    NAVIGATING= TH.state_nav,
    AVOIDING  = TH.state_avoid,
    ARRIVED   = TH.state_arr,
    ERROR     = TH.state_err,
}

local STATE_ICON = {
    IDLE      = "◌",
    NAVIGATING= "►",
    AVOIDING  = "⚠",
    ARRIVED   = "✓",
    ERROR     = "✗",
}

function GUI:drawStatus(s)
    local t = self.win_status
    local W = self.left_w - 2
    local function lv(row, label, value, vc)
        local llen = #label
        write_at(t, 2, row, label, TH.label, TH.panel_bg)
        write_at(t, 2 + llen, row, " "..tostring(value),
            vc or TH.value, TH.panel_bg)
        -- 清除行尾
        local used = 2 + llen + 1 + #tostring(value)
        local tail = W - used + 1
        if tail > 0 then
            write_at(t, used + 1, row, string.rep(" ", tail), TH.value, TH.panel_bg)
        end
    end

    local sc = STATE_COLOR[s.state] or TH.value
    local si = STATE_ICON[s.state]  or "?"

    local r = 2  -- 起始行（边框内）
    -- 状态行
    local state_str = si .. " " .. s.state
    lv(r,   "状 态  ", state_str, sc)
    -- 消息（截断）
    local msg = s.msg or ""
    if #msg > W - 3 then msg = msg:sub(1, W-6) .. "..." end
    write_at(t, 2, r+1, string.rep(" ", W), TH.label, TH.panel_bg)
    write_at(t, 2, r+1, msg, TH.label, TH.panel_bg)

    -- 分隔
    write_at(t, 2, r+2, string.rep("┄", W-1), TH.panel_border, TH.panel_bg)

    -- 位置
    lv(r+3, "位 置  ", string.format("X%-7.1f Y%-7.1f Z%-7.1f",
        s.pos.x, s.pos.y, s.pos.z), TH.value_hi)
    -- 速度
    local spd = s.velocity:length()
    lv(r+4, "速 度  ", string.format("%.2f m/s  (Vx%.1f Vy%.1f Vz%.1f)",
        spd, s.velocity.x, s.velocity.y, s.velocity.z))
    -- 偏航
    lv(r+5, "偏 航  ", string.format("%.1f °", s.yaw))

    -- 分隔
    write_at(t, 2, r+6, string.rep("┄", W-1), TH.panel_border, TH.panel_bg)

    -- 目标信息
    if s.target then
        lv(r+7, "目 标  ", string.format("X%-7.1f Y%-7.1f Z%-7.1f",
            s.target.x, s.target.y, s.target.z), TH.value_hi)
        lv(r+8, "距 离  ", string.format("%.1f 格", s.dist))
    else
        lv(r+7, "目 标  ", "-- 无 --",  TH.label)
        lv(r+8, "距 离  ", "0.0 格")
    end

    -- 统计
    write_at(t, 2, r+9, string.rep("┄", W-1), TH.panel_border, TH.panel_bg)
    lv(r+10, "总里程 ", string.format("%.1f 格", s.total_dist))
    lv(r+11, "用 时  ", string.format("%.1f 秒", s.elapsed))
end

-- ── 进度条渲染 ────────────────────────────────────────────────
function GUI:drawProgress(s)
    local t = self.term
    local W = self.W
    local pct = 0
    local label = "待机"

    if s.wp_total and s.wp_total > 0 then
        -- 以"已完成航点数 + 当前段进度"计算总进度
        local seg_done = math.max(0, s.wp_current - 1)
        local seg_pct  = 0
        if s.dist and s._seg_dist and s._seg_dist > 0 then
            seg_pct = 1.0 - clamp(s.dist / s._seg_dist, 0, 1)
        end
        pct = (seg_done + seg_pct) / s.wp_total
        label = string.format(" 航点 %d/%d  %.1f%%  剩余 %.1f 格 ",
            s.wp_current, s.wp_total, pct*100, s.dist or 0)
    elseif s.state == "ARRIVED" then
        pct   = 1.0
        label = string.format(" 已到达！里程 %.1f 格  耗时 %.1fs ",
            s.total_dist, s.elapsed)
    end

    draw_progress(t, 2, self.ROW_PROG, W-2, pct, label,
        colors.white, TH.progress_fg)
    -- 边框
    write_at(t, 1, self.ROW_PROG, "│", TH.panel_border, TH.bg)
    write_at(t, W, self.ROW_PROG, "│", TH.panel_border, TH.bg)
end

-- ── 航点列表渲染 ──────────────────────────────────────────────
function GUI:drawWaypoints(waypoints, current_idx)
    local t  = self.win_wp
    local W  = self.right_w - 2
    local H  = self.PANEL_H - 2
    -- 可见行数
    local max_rows = H - 1
    local total = #waypoints

    -- 滚动偏移：当前航点尽量在中间
    local offset = math.max(0, math.min(
        current_idx - math.floor(max_rows / 2),
        total - max_rows))
    if offset < 0 then offset = 0 end

    for i = 1, max_rows do
        local wi = i + offset
        local row = i + 1  -- 边框内起始行
        if wi <= total then
            local wp = waypoints[wi]
            local is_cur = (wi == current_idx)
            local is_done= (wi < current_idx)
            local fg = is_cur  and TH.wp_active
                    or is_done and TH.wp_done
                    or            TH.wp_pending
            local icon = is_cur  and "►"
                      or is_done and "✓"
                      or            "○"
            local line = string.format("%s[%2d] %8.1f %8.1f %8.1f",
                icon, wi, wp.x, wp.y, wp.z)
            if #line > W then line = line:sub(1, W) end
            write_at(t, 2, row, string.rep(" ", W), fg, TH.panel_bg)
            write_at(t, 2, row, line, fg, TH.panel_bg)
        else
            -- 清空多余行
            write_at(t, 2, row, string.rep(" ", W), TH.panel_bg, TH.panel_bg)
        end
    end

    -- 底部统计
    local stat = string.format(" 共 %d 个航点 ", total)
    write_at(t, 2, H+1, string.rep(" ", W), TH.label, TH.panel_bg)
    write_at(t, 2, H+1, stat, TH.label, TH.panel_bg)
end

-- ── 输入框 ────────────────────────────────────────────────────
function GUI:drawInput()
    local t = self.term
    local W = self.W
    local prompt = self.input_prompt
    local buf    = self.input_buf

    fill_rect(t, 1, self.ROW_INPUT, W, 1, TH.input_bg)
    write_at(t, 1, self.ROW_INPUT, prompt, TH.input_cursor, TH.input_bg)
    local px = 1 + #prompt
    -- 截断显示
    local max_show = W - px - 1
    local show = buf
    if #show > max_show then show = show:sub(#show - max_show + 1) end
    write_at(t, px, self.ROW_INPUT, show, TH.input_fg, TH.input_bg)
    -- 光标位置
    local cx = px + #show
    if cx <= W then
        write_at(t, cx, self.ROW_INPUT, "▌", TH.input_cursor, TH.input_bg)
    end
    -- 复位光标位置（防止乱跳）
    t.setCursorPos(cx, self.ROW_INPUT)
    t.setTextColor(TH.input_fg)
    t.setBackgroundColor(TH.input_bg)
end

-- ── 日志系统 ──────────────────────────────────────────────────
function GUI:log(msg, level)
    level = level or "INFO"
    local fg = TH.log_info
    local icon = "ℹ"
    if level == "OK"   then fg = TH.log_ok;   icon = "✓"
    elseif level == "WARN" then fg = TH.log_warn; icon = "⚠"
    elseif level == "ERR"  then fg = TH.log_err;  icon = "✗"
    end
    local time_str = string.format("[%05.1f]", os.clock() % 1000)
    table.insert(self.log_lines, { text=time_str.." "..icon.." "..msg, fg=fg })
    while #self.log_lines > self.log_max do
        table.remove(self.log_lines, 1)
    end
    self:drawLog()
end

function GUI:drawLog()
    local t = self.term
    local W = self.W
    for i, entry in ipairs(self.log_lines) do
        local row = self.ROW_LOG + i - 1
        if row <= self.H then
            fill_rect(t, 1, row, W, 1, TH.log_bg)
            local text = entry.text
            if #text > W then text = text:sub(1, W-1) .. "…" end
            write_at(t, 1, row, text, entry.fg, TH.log_bg)
        end
    end
    -- 补空白行
    for i = #self.log_lines + 1, self.log_max do
        local row = self.ROW_LOG + i - 1
        if row <= self.H then
            fill_rect(t, 1, row, W, 1, TH.log_bg)
        end
    end
end

-- ── 全量刷新 ──────────────────────────────────────────────────
function GUI:render(nav_status, waypoints, wp_current)
    self:drawStatus(nav_status)
    self:drawProgress(nav_status)
    self:drawWaypoints(waypoints, wp_current)
    self:drawInput()
end

-- ── 弹出对话框（模态输入）────────────────────────────────────
-- 在屏幕中央弹出一个小窗口，等待用户输入
-- title: 标题字符串
-- fields: { {label="X:", default="0"}, ... }
-- 返回: { field1_value, field2_value, ... } 或 nil（取消）
function GUI:dialog(title, fields)
    local t = self.term
    local W = self.W
    local dw = math.min(W - 4, 44)
    local dh = #fields * 2 + 4
    local dx = math.floor((W - dw) / 2) + 1
    local dy = math.floor((self.H - dh) / 2) + 1

    -- 背景遮罩
    local saved = {}
    for y = dy, dy+dh-1 do
        for x = dx, dx+dw-1 do
            -- CC:Tweaked 无法获取单元格，直接绘制
        end
    end

    -- 绘制对话框
    fill_rect(t, dx, dy, dw, dh, colors.navy or colors.blue)
    draw_box(t, dx, dy, dw, dh, title, colors.cyan, colors.white)

    local results = {}
    for i, field in ipairs(fields) do
        local fy = dy + 1 + (i-1)*2
        write_at(t, dx+2, fy,   field.label, colors.white, colors.blue)
        -- 输入框背景
        fill_rect(t, dx+2+#field.label+1, fy, dw-#field.label-4, 1, colors.lightGray)

        local buf = field.default or ""
        local inp_x = dx + 2 + #field.label + 1
        local inp_w = dw - #field.label - 5

        -- 简单行输入
        while true do
            -- 显示缓冲
            fill_rect(t, inp_x, fy, inp_w, 1, colors.lightGray)
            local show = buf
            if #show > inp_w - 1 then show = show:sub(#show - inp_w + 2) end
            write_at(t, inp_x, fy, show, colors.black, colors.lightGray)
            write_at(t, inp_x + #show, fy, "▌", colors.blue, colors.lightGray)
            t.setCursorPos(inp_x + #show, fy)

            local ev, p1 = os.pullEvent()
            if ev == "char" then
                buf = buf .. p1
            elseif ev == "key" then
                if p1 == keys.backspace and #buf > 0 then
                    buf = buf:sub(1, -2)
                elseif p1 == keys.enter then
                    break
                elseif p1 == keys.escape then
                    -- 取消
                    self:drawFrame()
                    return nil
                end
            end
        end
        results[i] = buf
    end

    -- 关闭对话框（重绘整个界面）
    self:drawFrame()
    return results
end

-- ── 鼠标点击检测 ──────────────────────────────────────────────
---检测点击是否命中某个按钮，返回 action 字符串或 nil
function GUI:hitButton(mx, my)
    local by = self.ROW_BTN
    if my ~= by and my ~= by+1 then return nil end
    for _, btn in ipairs(self.buttons) do
        if mx >= btn.x and mx < btn.x + btn.w then
            return btn.action
        end
    end
    return nil
end

return GUI
