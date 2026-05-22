-- =============================================================
--  autopilot/gui.lua
  -- TUI
--
  -- 5119 Advanced Computer
--  
  -- row 1
--  
  -- ()       ()               row 2-13
--  
  -- row 14
--  
  -- [GO][WP+][CLR][START][STOP][TUNE][?]           row 15-16
--  
  -- row 17
--  
  -- 2                                      row 18-19
--  
-- =============================================================

local GUI = {}
GUI.__index = GUI


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


local function draw_box(t, x, y, w, h, title, border_c, title_c)

    write_at(t, x,   y, "+", border_c, TH.panel_bg)
    write_at(t, x+1, y, string.rep("-", w-2), border_c, TH.panel_bg)
    write_at(t, x+w-1, y, "+", border_c, TH.panel_bg)

    write_at(t, x,   y+h-1, "+", border_c, TH.panel_bg)
    write_at(t, x+1, y+h-1, string.rep("-", w-2), border_c, TH.panel_bg)
    write_at(t, x+w-1, y+h-1, "+", border_c, TH.panel_bg)

    for dy = 1, h-2 do
        write_at(t, x,     y+dy, "|", border_c, TH.panel_bg)
        write_at(t, x+w-1, y+dy, "|", border_c, TH.panel_bg)

        write_at(t, x+1, y+dy, string.rep(" ", w-2), TH.value, TH.panel_bg)
    end

    if title then
        local tlen = #title
        local tx = x + math.floor((w - tlen - 2) / 2)
        write_at(t, tx, y, " "..title.." ", title_c or TH.panel_title, TH.panel_bg)
    end
end


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


  -- : { label, x, y, w, color, action_id }
local function make_buttons(W)
  -- 7
    local btns = {
        { label="  GO   ", color=TH.btn_go,   action="goto"  },
        { label=" WP+  ", color=TH.btn_active,action="wp_add"},
        { label=" CLR  ", color=TH.btn_bg,    action="wp_clear"},
        { label=" LIST ", color=TH.btn_bg,    action="wp_list"},
        { label=" START", color=TH.btn_go,    action="start" },
        { label=" STOP ", color=TH.btn_stop,  action="stop"  },
        { label=" HELP ", color=TH.btn_bg,    action="help"  },
    }

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

  -- GUI
function GUI.new()
    local W, H = term.getSize()
    local obj = setmetatable({
        W = W, H = H,
        term = term,

        win_status = nil,
        win_wp     = nil,
        win_log    = nil,
        win_input  = nil,

        ROW_TITLE  = 1,
        ROW_PANEL  = 2,
        ROW_PROG   = nil,
        ROW_BTN    = nil,
        ROW_INPUT  = nil,
        ROW_LOG    = nil,

        buttons    = nil,

        input_buf  = "",
        input_cb   = nil,  -- function(text)
        input_prompt = "> ",

        log_lines  = {},
        log_max    = 4,
        _last_state = nil,
        -- set true while a modal dialog is open; nav_loop skips render
        modal      = false,
    }, GUI)


  -- = H - (1) - (1) - (2) - (1) - (2) = H - 7
    local panel_h = H - 7
    if panel_h < 4 then panel_h = 4 end

    obj.PANEL_H    = panel_h
    obj.ROW_PROG   = 2 + panel_h
    obj.ROW_BTN    = obj.ROW_PROG + 1
    obj.ROW_INPUT  = obj.ROW_BTN + 2
    obj.ROW_LOG    = obj.ROW_INPUT + 1
    obj.log_max    = H - obj.ROW_LOG + 1
    if obj.log_max < 1 then obj.log_max = 1 end


    local left_w  = math.floor(W * 0.52)
    local right_w = W - left_w


    obj.win_status = window.create(term.current(),
        1, obj.ROW_PANEL, left_w, panel_h, true)
    obj.win_wp = window.create(term.current(),
        left_w + 1, obj.ROW_PANEL, right_w, panel_h, true)

    obj.left_w  = left_w
    obj.right_w = right_w
    obj.buttons = make_buttons(W)

    return obj
end

  -- +
function GUI:drawFrame()
    local t = self.term
    local W, H = self.W, self.H


    fill_rect(t, 1, 1, W, H, TH.bg)


    fill_rect(t, 1, 1, W, 1, TH.title_bg)
    local title = " [*] Create:Aeronautics PID Autopilot v1.0  |  NeoForge 1.21.1 + CC:Tweaked "
    local tx = math.max(1, math.floor((W - #title) / 2) + 1)
    write_at(t, tx, 1, title, TH.title_fg, TH.title_bg)


    draw_box(self.win_status, 1, 1, self.left_w, self.PANEL_H,
             "[ STATUS ]", TH.panel_border, TH.panel_title)

    draw_box(self.win_wp, 1, 1, self.right_w, self.PANEL_H,
             "[WAYPOINTS]", TH.panel_border, TH.panel_title)


    fill_rect(t, 1, self.ROW_BTN,   W, 1, TH.bar_bg)
    fill_rect(t, 1, self.ROW_BTN+1, W, 1, TH.bar_bg)
    self:drawButtons()


    fill_rect(t, 1, self.ROW_INPUT, W, 1, TH.input_bg)


    fill_rect(t, 1, self.ROW_LOG, W, self.log_max, TH.log_bg)

    write_at(t, 1, self.ROW_LOG - 1,
        "--- LOG " .. string.rep("-", W - 9), TH.panel_border, TH.bg)
end


function GUI:drawButtons(highlighted)
    local t = self.term
    local y = self.ROW_BTN
    for _, btn in ipairs(self.buttons) do
        local bg = (btn.action == highlighted) and colors.white or btn.color
        local fg = (btn.action == highlighted) and colors.black or TH.btn_fg
        fill_rect(t, btn.x, y, btn.w, 2, bg)
        write_at(t, btn.x, y,   btn.label, fg, bg)
  -- /
        local icon = ""
        if btn.action == "goto"    then icon = "  >>   "
        elseif btn.action == "wp_add"  then icon = "  (+)  "
        elseif btn.action == "wp_clear"then icon = "  (x)  "
        elseif btn.action == "wp_list" then icon = "  ===  "
        elseif btn.action == "start"   then icon = "  >>   "
        elseif btn.action == "stop"    then icon = "  [ ]  "
        elseif btn.action == "help"    then icon = "  ?    "
        end
        icon = icon:sub(1, btn.w)
        while #icon < btn.w do icon = icon .. " " end
        write_at(t, btn.x, y+1, icon, fg, bg)
    end
end


local STATE_COLOR = {
    IDLE   = TH.state_idle,
    HOVER  = TH.state_nav,
    FLY    = TH.state_nav,
    GOTO   = TH.state_nav,
    MANUAL = TH.state_avoid,
    ERROR  = TH.state_err,
}

local STATE_ICON = {
    IDLE   = ".",
    HOVER  = "^",
    FLY    = ">",
    GOTO   = "*",
    MANUAL = "M",
    ERROR  = "x",
}

function GUI:drawStatus(s)
    local t = self.win_status
    local W = self.left_w - 2
    local function lv(row, label, value, vc)
        local llen = #label
        write_at(t, 2, row, label, TH.label, TH.panel_bg)
        local vs = " " .. tostring(value)
        write_at(t, 2 + llen, row, vs, vc or TH.value, TH.panel_bg)
        local used = 2 + llen + #vs
        local tail = W - used + 1
        if tail > 0 then
            write_at(t, used + 1, row, string.rep(" ", tail), TH.value, TH.panel_bg)
        end
    end

    local mode = s.mode or "ERROR"
    local sc = STATE_COLOR[mode] or TH.value
    local si = STATE_ICON[mode]  or "?"

    local r = 2

    lv(r, "Mode:   ", si .. " " .. mode, sc)

    local msg = s.msg or ""
    if #msg > W - 2 then msg = msg:sub(1, W-5) .. "..." end
    write_at(t, 2, r+1, string.rep(" ", W), TH.label, TH.panel_bg)
    write_at(t, 2, r+1, msg, TH.label, TH.panel_bg)

    write_at(t, 2, r+2, string.rep("-", W-1), TH.panel_border, TH.panel_bg)

    -- Flight data
    local alt_c = TH.value_hi
    lv(r+3, "Alt:    ", string.format("%.1f blk%s",
        s.altitude or 0,
        s.tgt_alt and string.format("  ->%.0f", s.tgt_alt) or ""), alt_c)

    -- Show horizontal speed with vertical component in parentheses
    local spd_str = string.format("%.2f m/s",  s.horiz_speed or s.speed or 0)
    if (s.vert_speed or 0) ~= 0 then
        spd_str = spd_str .. string.format("  vt:%.1f", s.vert_speed)
    end
    if s.tgt_spd and s.tgt_spd > 0 then
        spd_str = spd_str .. string.format("  ->%.1f", s.tgt_spd)
    end
    lv(r+4, "Speed:  ", spd_str)

    lv(r+5, "Heading:", string.format("%.1f deg%s",
        s.heading or 0,
        s.tgt_hdg and string.format("  ->%.0f", s.tgt_hdg) or ""))

    write_at(t, 2, r+6, string.rep("-", W-1), TH.panel_border, TH.panel_bg)

    lv(r+7, "Pitch:  ", string.format("%.1f  Roll: %.1f",
        s.pitch or 0, s.roll or 0))

    -- DR position + odometer
    lv(r+8, "DR pos: ", string.format("dX:%.0f dZ:%.0f  odo:%.0f",
        s.dr_x or 0, s.dr_z or 0, s.dr_dist or 0), TH.value_hi)

    write_at(t, 2, r+9, string.rep("-", W-1), TH.panel_border, TH.panel_bg)

    -- Sensor status line
    local sens_str = s.sensors or "--"
    if #sens_str > W - 2 then sens_str = sens_str:sub(1, W-5) .. "..." end
    write_at(t, 2, r+10, string.rep(" ", W), TH.label, TH.panel_bg)
    write_at(t, 2, r+10, sens_str, TH.label, TH.panel_bg)

    lv(r+11, "Time:   ", string.format("%.1fs", s.elapsed or 0))
end


function GUI:drawProgress(s)
    local t = self.term
    local W = self.W
    local pct = 0
    local label = "Standby"

    if s.wp_total and s.wp_total > 0 then
  -- " + "
        local seg_done = math.max(0, s.wp_current - 1)
        local seg_pct  = 0
        if s.dist and s._seg_dist and s._seg_dist > 0 then
            seg_pct = 1.0 - clamp(s.dist / s._seg_dist, 0, 1)
        end
        pct = (seg_done + seg_pct) / s.wp_total
        label = string.format(" WP %d/%d  %.1f%%  Rem %.1f blk ",
            s.wp_current, s.wp_total, pct*100, s.dist or 0)
    elseif s.state == "ARRIVED" then
        pct   = 1.0
        label = string.format(" Arrived! ODO %.1f blk  %.1fs ",
            s.total_dist, s.elapsed)
    end

    draw_progress(t, 2, self.ROW_PROG, W-2, pct, label,
        colors.white, TH.progress_fg)

    write_at(t, 1, self.ROW_PROG, "|", TH.panel_border, TH.bg)
    write_at(t, W, self.ROW_PROG, "|", TH.panel_border, TH.bg)
end


function GUI:drawWaypoints(waypoints, current_idx)
    local t  = self.win_wp
    local W  = self.right_w - 2
    local H  = self.PANEL_H - 2

    local max_rows = H - 1
    local total = #waypoints


    local offset = math.max(0, math.min(
        current_idx - math.floor(max_rows / 2),
        total - max_rows))
    if offset < 0 then offset = 0 end

    for i = 1, max_rows do
        local wi = i + offset
        local row = i + 1
        if wi <= total then
            local wp = waypoints[wi]
            local is_cur = (wi == current_idx)
            local is_done= (wi < current_idx)
            local fg = is_cur  and TH.wp_active
                    or is_done and TH.wp_done
                    or            TH.wp_pending
            local icon = is_cur  and ">"
                      or is_done and "+"
                      or            "o"
            local line = string.format("%s[%2d] %8.1f %8.1f %8.1f",
                icon, wi, wp.x, wp.y, wp.z)
            if #line > W then line = line:sub(1, W) end
            write_at(t, 2, row, string.rep(" ", W), fg, TH.panel_bg)
            write_at(t, 2, row, line, fg, TH.panel_bg)
        else

            write_at(t, 2, row, string.rep(" ", W), TH.panel_bg, TH.panel_bg)
        end
    end


    local stat = string.format(" Total: %d WPs ", total)
    write_at(t, 2, H+1, string.rep(" ", W), TH.label, TH.panel_bg)
    write_at(t, 2, H+1, stat, TH.label, TH.panel_bg)
end


function GUI:drawInput()
    local t = self.term
    local W = self.W
    local prompt = self.input_prompt
    local buf    = self.input_buf

    fill_rect(t, 1, self.ROW_INPUT, W, 1, TH.input_bg)
    write_at(t, 1, self.ROW_INPUT, prompt, TH.input_cursor, TH.input_bg)
    local px = 1 + #prompt

    local max_show = W - px - 1
    local show = buf
    if #show > max_show then show = show:sub(#show - max_show + 1) end
    write_at(t, px, self.ROW_INPUT, show, TH.input_fg, TH.input_bg)

    local cx = px + #show
    if cx <= W then
        write_at(t, cx, self.ROW_INPUT, "_", TH.input_cursor, TH.input_bg)
    end

    t.setCursorPos(cx, self.ROW_INPUT)
    t.setTextColor(TH.input_fg)
    t.setBackgroundColor(TH.input_bg)
    t.setCursorBlink(true)
end


function GUI:log(msg, level)
    level = level or "INFO"
    local fg = TH.log_info
    local icon = "i"
    if level == "OK"   then fg = TH.log_ok;   icon = "o"
    elseif level == "WARN" then fg = TH.log_warn; icon = "!"
    elseif level == "ERR"  then fg = TH.log_err;  icon = "x"
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
            if #text > W then text = text:sub(1, W-1) .. "..." end
            write_at(t, 1, row, text, entry.fg, TH.log_bg)
        end
    end

    for i = #self.log_lines + 1, self.log_max do
        local row = self.ROW_LOG + i - 1
        if row <= self.H then
            fill_rect(t, 1, row, W, 1, TH.log_bg)
        end
    end
end


function GUI:render(nav_status, waypoints, wp_current)
    self:drawStatus(nav_status)
    self:drawProgress(nav_status)
    self:drawWaypoints(waypoints, wp_current)
    self:drawInput()
end



-- title: string
-- fields: { {label="X:", default="0"}, ... }
-- returns: { field1_value, field2_value, ... }  or nil if cancelled
function GUI:dialog(title, fields)
    self.modal = true

    local t  = self.term
    local W  = self.W
    local H  = self.H
    local dw = math.min(W - 4, 36)
    local dh = #fields * 3 + 4
    local dx = math.floor((W - dw) / 2) + 1
    local dy = math.max(2, math.floor((H - dh) / 2))

    -- Draw dialog directly on the main terminal (no window.create)
    local function draw_dialog(focused_field, buf_map)
        t.setCursorBlink(false)
        -- background fill
        for row = 0, dh - 1 do
            t.setCursorPos(dx, dy + row)
            t.setBackgroundColor(colors.blue)
            t.setTextColor(colors.white)
            t.write(string.rep(" ", dw))
        end
        -- top border
        t.setCursorPos(dx, dy)
        t.setTextColor(colors.cyan)
        t.setBackgroundColor(colors.blue)
        t.write("+" .. string.rep("-", dw - 2) .. "+")
        -- bottom border
        t.setCursorPos(dx, dy + dh - 1)
        t.write("+" .. string.rep("-", dw - 2) .. "+")
        -- side borders
        for row = 1, dh - 2 do
            t.setCursorPos(dx,          dy + row)
            t.setTextColor(colors.cyan)
            t.setBackgroundColor(colors.blue)
            t.write("|")
            t.setCursorPos(dx + dw - 1, dy + row)
            t.write("|")
        end
        -- title centered on top border
        local tstr = " " .. title .. " "
        local tx = dx + math.floor((dw - #tstr) / 2)
        t.setCursorPos(tx, dy)
        t.setTextColor(colors.white)
        t.setBackgroundColor(colors.blue)
        t.write(tstr)
        -- hint on bottom border
        local hint = "Enter=OK  Esc=Cancel"
        local hx = dx + math.floor((dw - #hint) / 2)
        t.setCursorPos(hx, dy + dh - 1)
        t.setTextColor(colors.yellow)
        t.setBackgroundColor(colors.blue)
        t.write(hint)
        -- fields
        local inp_w = dw - 4
        for i, field in ipairs(fields) do
            local fy = dy + 1 + (i - 1) * 3
            -- label row
            t.setCursorPos(dx + 1, fy)
            t.setTextColor(colors.white)
            t.setBackgroundColor(colors.blue)
            t.write(field.label)
            -- input row
            local is_focused = (i == focused_field)
            local buf = buf_map[i] or ""
            local show = buf
            if #show > inp_w - 1 then show = show:sub(#show - inp_w + 2) end
            local box_bg = is_focused and colors.lightGray or colors.gray
            t.setCursorPos(dx + 1, fy + 1)
            t.setBackgroundColor(box_bg)
            t.setTextColor(colors.black)
            local content = show .. (is_focused and "_" or " ")
            local pad = inp_w - #content
            t.write(" " .. content .. string.rep(" ", math.max(0, pad)))
        end
        -- place real cursor at focused input
        local cur_i   = focused_field
        local cur_fy  = dy + 1 + (cur_i - 1) * 3 + 1
        local cur_buf = buf_map[cur_i] or ""
        local cur_show = cur_buf
        if #cur_show > inp_w - 1 then cur_show = cur_show:sub(#cur_show - inp_w + 2) end
        t.setCursorPos(dx + 1 + #cur_show + 1, cur_fy)
        t.setBackgroundColor(colors.lightGray)
        t.setTextColor(colors.black)
        t.setCursorBlink(true)
    end

    local focused = 1
    local bufs = {}
    for i, field in ipairs(fields) do
        bufs[i] = field.default or ""
    end

    draw_dialog(focused, bufs)

    local result = nil
    while true do
        local ev, p1 = os.pullEvent()
        if ev == "char" then
            bufs[focused] = bufs[focused] .. p1
            draw_dialog(focused, bufs)
        elseif ev == "key" then
            if p1 == keys.backspace then
                if #bufs[focused] > 0 then
                    bufs[focused] = bufs[focused]:sub(1, -2)
                end
            elseif p1 == keys.enter then
                if focused < #fields then
                    focused = focused + 1
                else
                    result = bufs
                    break
                end
            elseif p1 == keys.tab then
                focused = (focused % #fields) + 1
            elseif p1 == keys.escape then
                break
            end
            draw_dialog(focused, bufs)
        end
        -- ignore all other events (timer, mouse, etc.) while dialog is open
    end

    -- Full repaint to erase dialog remnants
    t.setCursorBlink(false)
    self.modal = false
    self:drawFrame()
    self:drawLog()
    self:drawInput()

    return result
end


  -- - action  nil
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
