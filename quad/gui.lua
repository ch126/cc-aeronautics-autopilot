-- =============================================================
--  quad/gui.lua
--  Quadrotor TUI  (mirrors autopilot/gui.lua style)
--
--  Layout (51x19 Advanced Computer):
--  +--[STATUS]----+--[MOTORS]---+   row 2..H-7
--  | AHI compass  | RPM bars    |
--  | alt/spd/yaw  | M1 M2 M3 M4 |
--  +--------------+-------------+
--  [=====progress bar=====]         row H-5
--  [ARM][DISARM][HOVER][LAND][HELP] row H-4..H-3
--  > input___                       row H-2
--  --- LOG ---                      row H-1..H
-- =============================================================

local GUI = {}
GUI.__index = GUI

-- ── theme ─────────────────────────────────────────────────────
local TH = {
    bg           = colors.black,
    title_bg     = colors.blue,
    title_fg     = colors.white,
    panel_bg     = colors.black,
    panel_border = colors.gray,
    panel_title  = colors.cyan,
    label        = colors.gray,
    value        = colors.white,
    value_hi     = colors.yellow,
    armed_on     = colors.lime,
    armed_off    = colors.red,
    btn_bg       = colors.gray,
    btn_fg       = colors.white,
    btn_arm      = colors.green,
    btn_disarm   = colors.red,
    btn_active   = colors.blue,
    progress_bg  = colors.gray,
    progress_fg  = colors.lime,
    log_bg       = colors.black,
    log_ok       = colors.lime,
    log_warn     = colors.yellow,
    log_err      = colors.red,
    log_info     = colors.lightGray,
    input_bg     = colors.gray,
    input_fg     = colors.white,
    input_cursor = colors.yellow,
    bar_bg       = colors.gray,
    rpm_lo       = colors.gray,
    rpm_mid      = colors.lime,
    rpm_hi       = colors.yellow,
    rpm_max      = colors.red,
}

-- ── primitives ────────────────────────────────────────────────
local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end

local function write_at(t, x, y, text, fg, bg)
    t.setCursorPos(x, y)
    if fg then t.setTextColor(fg) end
    if bg then t.setBackgroundColor(bg) end
    t.write(text)
end

local function fill_rect(t, x, y, w, h, bg)
    t.setBackgroundColor(bg)
    local row = string.rep(" ", w)
    for dy = 0, h - 1 do
        t.setCursorPos(x, y + dy)
        t.write(row)
    end
end

local function draw_box(t, x, y, w, h, title, border_c, title_c)
    border_c = border_c or TH.panel_border
    write_at(t, x,       y,       "+" .. string.rep("-", w-2) .. "+", border_c, TH.panel_bg)
    write_at(t, x,       y+h-1,   "+" .. string.rep("-", w-2) .. "+", border_c, TH.panel_bg)
    for dy = 1, h - 2 do
        write_at(t, x,     y+dy, "|", border_c, TH.panel_bg)
        write_at(t, x+w-1, y+dy, "|", border_c, TH.panel_bg)
        write_at(t, x+1,   y+dy, string.rep(" ", w-2), TH.value, TH.panel_bg)
    end
    if title then
        local tx = x + math.floor((w - #title - 2) / 2)
        write_at(t, tx, y, " " .. title .. " ", title_c or TH.panel_title, TH.panel_bg)
    end
end

-- ── blit helpers ──────────────────────────────────────────────
local color_char = {
    [colors.black]     = "0", [colors.white]     = "f",
    [colors.orange]    = "1", [colors.magenta]   = "2",
    [colors.lightBlue] = "3", [colors.yellow]    = "4",
    [colors.lime]      = "5", [colors.pink]       = "6",
    [colors.gray]      = "7", [colors.lightGray] = "8",
    [colors.cyan]      = "9", [colors.purple]    = "a",
    [colors.blue]      = "b", [colors.brown]     = "c",
    [colors.green]     = "d", [colors.red]        = "e",
}
local function cc(col) return color_char[col] or "f" end
local function gstr(c, n)
    if n <= 0 then return "" end
    return string.rep(c, n)
end
local function blit_at(t, x, y, text, fg_col, bg_col)
    t.setCursorPos(x, y)
    local n = #text
    t.blit(text, gstr(cc(fg_col), n), gstr(cc(bg_col), n))
end

-- ── AHI ───────────────────────────────────────────────────────
local function draw_ahi(t, x0, y0, w, h, pitch, roll)
    local sky_c = colors.blue
    local gnd_c = colors.brown
    local fg_c  = colors.white
    local mid_y = h / 2
    local pitch_off = math.floor(pitch / 90 * mid_y + 0.5)
    pitch_off = clamp(pitch_off, -(h-1), h-1)
    local roll_rad = math.rad(roll)
    local half_w   = (w - 2) / 2
    local roll_dy  = clamp(math.floor(math.tan(roll_rad) * half_w + 0.5), -(h-1), h-1)

    for dy = 1, h do
        local horizon_row = mid_y - pitch_off
        local col = dy <= horizon_row and sky_c or gnd_c
        write_at(t, x0, y0 + dy - 1, string.rep(" ", w), fg_c, col)
    end
    local hl = clamp(math.floor(mid_y - pitch_off), 1, h)
    for dx = 0, math.floor(half_w) - 1 do
        local ly = clamp(hl + math.floor(roll_dy * dx / half_w + 0.5), 1, h)
        local col = (ly <= mid_y - pitch_off) and sky_c or gnd_c
        blit_at(t, x0 + dx, y0 + ly - 1, "-", fg_c, col)
    end
    for dx = 1, math.floor(half_w) do
        local ly = clamp(hl - math.floor(roll_dy * dx / half_w + 0.5), 1, h)
        local col = (ly <= mid_y - pitch_off) and sky_c or gnd_c
        blit_at(t, x0 + math.floor(half_w) + dx, y0 + ly - 1, "-", fg_c, col)
    end
    local cx = x0 + math.floor(w / 2) - 1
    local cy = y0 + math.floor(h / 2) - 1
    blit_at(t, cx - 2, cy, " -+- ", fg_c, TH.panel_bg)
end

-- ── Compass ───────────────────────────────────────────────────
local function draw_compass(t, x, y, w, heading, fg_c, bg_c)
    fg_c = fg_c or colors.white
    bg_c = bg_c or TH.title_bg
    local marks = {}
    for i = 0, 35 do
        local d = i * 10
        if     d == 0   then marks[i+1] = "N"
        elseif d == 90  then marks[i+1] = "E"
        elseif d == 180 then marks[i+1] = "S"
        elseif d == 270 then marks[i+1] = "W"
        else                 marks[i+1] = "|"
        end
    end
    local mid = math.floor(w / 2)
    local buf = {}
    for i = 1, w do
        local od = (i - mid - 1) * (360 / w)
        local idx = math.floor((heading + od) / 10 + 0.5) % 36 + 1
        buf[i] = marks[idx]
    end
    local str  = table.concat(buf)
    local pre  = str:sub(1, mid - 1)
    local mark = str:sub(mid, mid)
    local post = str:sub(mid + 1)
    t.setCursorPos(x, y)
    t.blit(pre,  gstr(cc(fg_c), #pre),  gstr(cc(bg_c), #pre))
    t.blit(mark, cc(colors.yellow),      cc(colors.red))
    t.blit(post, gstr(cc(fg_c), #post), gstr(cc(bg_c), #post))
end

-- ── RPM bar ───────────────────────────────────────────────────
local function draw_rpm_bar(t, x, y, w, rpm, rpm_max, label)
    local pct   = clamp(rpm / math.max(rpm_max, 1), 0, 1)
    local filled = math.floor((w - 4) * pct)
    local empty  = (w - 4) - filled
    local bar_fg = pct < 0.5 and TH.rpm_mid
               or  pct < 0.8 and TH.rpm_hi
               or               TH.rpm_max

    write_at(t, x, y, string.format("%-2s", label), TH.label, TH.panel_bg)
    t.setCursorPos(x + 2, y)
    t.setBackgroundColor(bar_fg)
    t.write(string.rep("|", filled))
    t.setBackgroundColor(TH.rpm_lo)
    t.write(string.rep(".", empty))
    t.setBackgroundColor(TH.panel_bg)
    t.setTextColor(TH.value)
    t.write(string.format("%3d", math.floor(rpm)))
end

-- ── Buttons ───────────────────────────────────────────────────
local function make_buttons(W)
    local btns = {
        { label=" ARM   ", color=TH.btn_arm,    action="arm"    },
        { label=" DISARM", color=TH.btn_disarm, action="disarm" },
        { label=" HOVER ", color=TH.btn_active, action="hover"  },
        { label=" LAND  ", color=TH.btn_active, action="land"   },
        { label=" GOTO  ", color=TH.btn_bg,     action="goto"   },
        { label=" HELP  ", color=TH.btn_bg,     action="help"   },
    }
    local total_w = 0
    for _, b in ipairs(btns) do total_w = total_w + #b.label + 1 end
    local gap = math.floor((W - total_w) / (#btns + 1))
    local cx = gap + 1
    for _, b in ipairs(btns) do
        b.x = cx
        b.w = #b.label
        cx  = cx + b.w + 1 + gap
    end
    return btns
end

-- ── Constructor ───────────────────────────────────────────────
function GUI.new()
    local W, H = term.getSize()

    -- panel height = rows between title and progress bar
    local panel_h = H - 7
    if panel_h < 4 then panel_h = 4 end

    local obj = setmetatable({
        W = W, H = H,
        term = term,
        PANEL_H    = panel_h,
        ROW_PANEL  = 2,
        ROW_PROG   = 2 + panel_h,
        ROW_BTN    = 2 + panel_h + 1,
        ROW_INPUT  = 2 + panel_h + 3,
        ROW_LOG    = 2 + panel_h + 4,
        log_lines  = {},
        log_max    = H - (2 + panel_h + 4) + 1,
        input_buf  = "",
        input_cb   = nil,
        input_prompt = "> ",
        buttons    = nil,
        modal      = false,
        win_status = nil,
        win_motors = nil,
    }, GUI)

    if obj.log_max < 1 then obj.log_max = 1 end

    local left_w  = math.floor(W * 0.56)
    local right_w = W - left_w
    obj.left_w  = left_w
    obj.right_w = right_w

    obj.win_status = window.create(term.current(),
        1, obj.ROW_PANEL, left_w, panel_h, true)
    obj.win_motors = window.create(term.current(),
        left_w + 1, obj.ROW_PANEL, right_w, panel_h, true)

    obj.buttons = make_buttons(W)
    return obj
end

-- ── Frame ─────────────────────────────────────────────────────
function GUI:drawFrame()
    local t = self.term
    local W, H = self.W, self.H

    fill_rect(t, 1, 1, W, H, TH.bg)

    fill_rect(t, 1, 1, W, 1, TH.title_bg)
    local title = " [Q] Create:Aeronautics Quad Flight Controller "
    local tx = math.max(1, math.floor((W - #title) / 2) + 1)
    write_at(t, tx, 1, title, TH.title_fg, TH.title_bg)

    draw_box(self.win_status, 1, 1, self.left_w,  self.PANEL_H, "STATUS",  TH.panel_border, TH.panel_title)
    draw_box(self.win_motors, 1, 1, self.right_w, self.PANEL_H, "MOTORS",  TH.panel_border, TH.panel_title)

    fill_rect(t, 1, self.ROW_BTN,   W, 1, TH.bar_bg)
    fill_rect(t, 1, self.ROW_BTN+1, W, 1, TH.bar_bg)
    self:drawButtons()

    fill_rect(t, 1, self.ROW_INPUT, W, 1, TH.input_bg)
    fill_rect(t, 1, self.ROW_LOG,   W, self.log_max, TH.log_bg)
    write_at(t, 1, self.ROW_LOG - 1,
        "--- LOG " .. string.rep("-", W - 9), TH.panel_border, TH.bg)
end

-- ── Buttons ───────────────────────────────────────────────────
function GUI:drawButtons(highlighted)
    local t = self.term
    local y = self.ROW_BTN
    local icons = {
        arm="  /\\  ", disarm=" STOP ", hover="  --  ",
        land="  \\/  ", ["goto"]="  >>  ", help="  ?   "
    }
    for _, btn in ipairs(self.buttons) do
        local hl = (btn.action == highlighted)
        local bg = hl and colors.white or btn.color
        local fg = hl and colors.black or TH.btn_fg
        fill_rect(t, btn.x, y, btn.w, 2, bg)
        write_at(t, btn.x, y,   btn.label, fg, bg)
        local icon = (icons[btn.action] or string.rep(" ", btn.w)):sub(1, btn.w)
        while #icon < btn.w do icon = icon .. " " end
        write_at(t, btn.x, y+1, icon, fg, bg)
    end
end

-- ── Status panel (left) ───────────────────────────────────────
-- s fields: armed, mode, pitch, roll, yaw, alt, climb, target_alt,
--           target_yaw, x, z, sensors, elapsed
function GUI:drawStatus(s)
    local t  = self.win_status
    local CW = self.left_w - 2
    local r  = 2

    local function lv(row, label, val, vc)
        write_at(t, 2, row, label, TH.label, TH.panel_bg)
        local vs = " " .. tostring(val)
        if #vs > CW - #label then vs = vs:sub(1, CW - #label) end
        write_at(t, 2 + #label, row, vs, vc or TH.value, TH.panel_bg)
        local used = #label + #vs
        if used < CW then
            write_at(t, 2+used, row, string.rep(" ", CW-used), TH.value, TH.panel_bg)
        end
    end

    -- Row r+0: Armed state + mode
    local armed    = s.armed and "ARMED" or "DISARMED"
    local armed_c  = s.armed and TH.armed_on or TH.armed_off
    local mode_str = (s.mode or "IDLE")
    lv(r, "State:", armed .. "  " .. mode_str, armed_c)

    -- Row r+1: Alt + climb rate
    local alt_str = string.format("Alt:%5.1fm  Clmb:%+.1f", s.alt or 0, s.climb or 0)
    write_at(t, 2, r+1, alt_str .. string.rep(" ", CW - 1 - #alt_str), TH.value_hi, TH.panel_bg)

    -- Row r+2: Target alt / yaw
    local tgt_str = string.format("TgtAlt:%4.1f  TgtYaw:%5.1f",
        s.target_alt or 0, s.target_yaw or 0)
    if #tgt_str > CW - 1 then tgt_str = tgt_str:sub(1, CW-1) end
    write_at(t, 2, r+2, tgt_str .. string.rep(" ", CW - 1 - #tgt_str), TH.label, TH.panel_bg)

    -- Row r+3: Compass strip
    draw_compass(t, 2, r+3, CW - 1, s.yaw or 0, TH.title_fg, TH.title_bg)

    -- Rows r+4 ~ r+6: AHI (3 rows)
    local ahi_w = CW - 1
    draw_ahi(t, 2, r+4, ahi_w, 3, s.pitch or 0, s.roll or 0)

    -- Row r+7: Pitch / Roll / Yaw numeric
    local pry = string.format("P:%+5.1f  R:%+5.1f  Y:%5.1f",
        s.pitch or 0, s.roll or 0, s.yaw or 0)
    if #pry > CW - 1 then pry = pry:sub(1, CW-1) end
    write_at(t, 2, r+7, pry .. string.rep(" ", CW - 1 - #pry), TH.value, TH.panel_bg)

    -- Row r+8: GPS pos or "--"
    local pos_str
    if s.x then
        pos_str = string.format("GPS X:%.0f  Z:%.0f", s.x, s.z)
    else
        pos_str = "GPS: --"
    end
    write_at(t, 2, r+8, pos_str .. string.rep(" ", CW - 1 - #pos_str), TH.label, TH.panel_bg)

    -- Row r+9: sensors / uptime
    local sens_str = string.format("%s  t:%.0fs", s.sensors or "--", s.elapsed or 0)
    if #sens_str > CW - 1 then sens_str = sens_str:sub(1, CW-4) .. "..." end
    write_at(t, 2, r+9, sens_str .. string.rep(" ", CW - 1 - #sens_str), TH.label, TH.panel_bg)
end

-- ── Motor panel (right) ───────────────────────────────────────
-- motors: { fl, fr, br, bl }  (RPM values)
-- rpm_max: max RPM for scaling
function GUI:drawMotors(motors, rpm_max)
    local t   = self.win_motors
    local W   = self.right_w - 2
    rpm_max   = rpm_max or 256
    local r   = 2

    -- Layout: diagram top, then 4 bars
    --   FL  FR
    --    X
    --   BL  BR
    local cw2 = math.floor(W / 2)

    -- motor diagram
    local fl_c = (motors and motors.fl or 0) > 0 and TH.armed_on or TH.label
    local fr_c = (motors and motors.fr or 0) > 0 and TH.armed_on or TH.label
    local bl_c = (motors and motors.bl or 0) > 0 and TH.armed_on or TH.label
    local br_c = (motors and motors.br or 0) > 0 and TH.armed_on or TH.label

    write_at(t, 2,      r,   "M1(FL)", fl_c, TH.panel_bg)
    write_at(t, 2+cw2,  r,   "M2(FR)", fr_c, TH.panel_bg)
    write_at(t, 2+math.floor(cw2*0.4), r+1, "\\  /",  TH.label, TH.panel_bg)
    write_at(t, 2+math.floor(cw2*0.4), r+2, "/  \\",  TH.label, TH.panel_bg)
    write_at(t, 2,      r+3, "M4(BL)", bl_c, TH.panel_bg)
    write_at(t, 2+cw2,  r+3, "M3(BR)", br_c, TH.panel_bg)

    -- separator
    write_at(t, 2, r+4, string.rep("-", W-1), TH.panel_border, TH.panel_bg)

    -- RPM bars
    local bar_w = W - 1
    local rpm   = motors or {}
    draw_rpm_bar(t, 2, r+5, bar_w, rpm.fl or 0, rpm_max, "FL")
    draw_rpm_bar(t, 2, r+6, bar_w, rpm.fr or 0, rpm_max, "FR")
    draw_rpm_bar(t, 2, r+7, bar_w, rpm.br or 0, rpm_max, "BR")
    draw_rpm_bar(t, 2, r+8, bar_w, rpm.bl or 0, rpm_max, "BL")

    -- throttle / armed indicator
    local thr = rpm.throttle or 0
    local thr_pct = clamp(thr / math.max(rpm_max, 1), 0, 1) * 100
    write_at(t, 2, r+9,
        string.format("Thr: %3d%%  (%3d RPM)", math.floor(thr_pct), math.floor(thr)),
        TH.value_hi, TH.panel_bg)
    -- pad rest of line
    local thr_str = string.format("Thr: %3d%%  (%3d RPM)", math.floor(thr_pct), math.floor(thr))
    if #thr_str < W - 1 then
        write_at(t, 2 + #thr_str, r+9,
            string.rep(" ", W - 1 - #thr_str), TH.value, TH.panel_bg)
    end
end

-- ── Throttle progress bar ─────────────────────────────────────
function GUI:drawProgress(s)
    local t     = self.term
    local W     = self.W
    local armed = s.armed
    local thr   = s.throttle or 0
    local rpm_max = s.rpm_max or 256
    local pct   = armed and clamp(thr / rpm_max, 0, 1) or 0
    local lbl   = armed
        and string.format(" ARMED  Thr:%.0f  Alt:%.1fm->%.1fm  Yaw:%.1f ",
                thr, s.alt or 0, s.target_alt or 0, s.yaw or 0)
        or " DISARMED "

    -- draw bar
    local bw     = W - 2
    local filled = math.floor(bw * pct)
    local empty  = bw - filled
    local bar_fg = pct < 0.5 and TH.progress_fg
               or  pct < 0.8 and TH.rpm_hi
               or               TH.rpm_max

    t.setCursorPos(2, self.ROW_PROG)
    t.setBackgroundColor(bar_fg)
    t.write(string.rep(" ", filled))
    t.setBackgroundColor(TH.progress_bg)
    t.write(string.rep(" ", empty))

    -- overlay label
    if #lbl > bw then lbl = lbl:sub(1, bw) end
    local lx = 2 + math.floor((bw - #lbl) / 2)
    local ll = filled - (lx - 2)
    local t1 = lbl:sub(1, math.max(0, ll))
    local t2 = lbl:sub(math.max(1, ll + 1))
    if #t1 > 0 then
        write_at(t, lx, self.ROW_PROG, t1, colors.white, bar_fg)
    end
    if #t2 > 0 then
        write_at(t, lx + #t1, self.ROW_PROG, t2, colors.white, TH.progress_bg)
    end
    write_at(t, 1,   self.ROW_PROG, "|", TH.panel_border, TH.bg)
    write_at(t, W,   self.ROW_PROG, "|", TH.panel_border, TH.bg)
end

-- ── Input line ────────────────────────────────────────────────
function GUI:drawInput()
    local t = self.term
    local W = self.W
    fill_rect(t, 1, self.ROW_INPUT, W, 1, TH.input_bg)
    write_at(t, 1, self.ROW_INPUT, self.input_prompt, TH.input_cursor, TH.input_bg)
    local px  = 1 + #self.input_prompt
    local max_show = W - px - 1
    local show = self.input_buf
    if #show > max_show then show = show:sub(#show - max_show + 1) end
    write_at(t, px, self.ROW_INPUT, show, TH.input_fg, TH.input_bg)
    local cx = px + #show
    if cx <= W then
        write_at(t, cx, self.ROW_INPUT, "_", TH.input_cursor, TH.input_bg)
    end
    t.setCursorPos(cx, self.ROW_INPUT)
    t.setCursorBlink(true)
end

-- ── Log ───────────────────────────────────────────────────────
function GUI:log(msg, level)
    level = level or "INFO"
    local fg   = TH.log_info
    local icon = "i"
    if level == "OK"   then fg = TH.log_ok;   icon = "o"
    elseif level == "WARN" then fg = TH.log_warn; icon = "!"
    elseif level == "ERR"  then fg = TH.log_err;  icon = "x"
    end
    local ts = string.format("[%05.1f]", os.clock() % 1000)
    table.insert(self.log_lines, { text = ts .. " " .. icon .. " " .. msg, fg = fg })
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
            if #text > W then text = text:sub(1, W-1) .. ">" end
            write_at(t, 1, row, text, entry.fg, TH.log_bg)
        end
    end
    for i = #self.log_lines + 1, self.log_max do
        local row = self.ROW_LOG + i - 1
        if row <= self.H then fill_rect(t, 1, row, W, 1, TH.log_bg) end
    end
end

-- ── Full render ───────────────────────────────────────────────
-- status:  { armed, mode, pitch, roll, yaw, alt, climb, target_alt,
--            target_yaw, x, z, sensors, elapsed, throttle, rpm_max }
-- motors:  { fl, fr, br, bl, throttle }
function GUI:render(status, motors)
    self:drawStatus(status)
    self:drawMotors(motors, status.rpm_max)
    self:drawProgress(status)
    self:drawInput()
end

-- ── hit-test buttons ──────────────────────────────────────────
function GUI:hitButton(mx, my)
    local by = self.ROW_BTN
    if my ~= by and my ~= by + 1 then return nil end
    for _, btn in ipairs(self.buttons) do
        if mx >= btn.x and mx < btn.x + btn.w then
            return btn.action
        end
    end
    return nil
end

-- ── Simple input dialog ───────────────────────────────────────
function GUI:dialog(title, fields)
    self.modal = true
    local t  = self.term
    local W, H = self.W, self.H
    local dw = math.min(W - 4, 38)
    local dh = #fields * 3 + 4
    local dx = math.floor((W - dw) / 2) + 1
    local dy = math.max(2, math.floor((H - dh) / 2))
    local inp_w = dw - 4

    local function redraw(fi, bufs)
        t.setCursorBlink(false)
        for row = 0, dh - 1 do
            t.setCursorPos(dx, dy + row)
            t.setBackgroundColor(colors.blue)
            t.setTextColor(colors.white)
            t.write(string.rep(" ", dw))
        end
        t.setCursorPos(dx, dy)
        t.setTextColor(colors.cyan)
        t.setBackgroundColor(colors.blue)
        t.write("+" .. string.rep("-", dw-2) .. "+")
        t.setCursorPos(dx, dy + dh - 1)
        t.write("+" .. string.rep("-", dw-2) .. "+")
        for row = 1, dh-2 do
            t.setCursorPos(dx,          dy+row)
            t.setTextColor(colors.cyan)
            t.setBackgroundColor(colors.blue)
            t.write("|")
            t.setCursorPos(dx+dw-1, dy+row)
            t.write("|")
        end
        local tstr = " " .. title .. " "
        t.setCursorPos(dx + math.floor((dw - #tstr) / 2), dy)
        t.setTextColor(colors.white)
        t.setBackgroundColor(colors.blue)
        t.write(tstr)
        local hint = "Enter=OK  Esc=Cancel"
        t.setCursorPos(dx + math.floor((dw - #hint) / 2), dy + dh - 1)
        t.setTextColor(colors.yellow)
        t.setBackgroundColor(colors.blue)
        t.write(hint)
        for i, field in ipairs(fields) do
            local fy = dy + 1 + (i-1) * 3
            t.setCursorPos(dx+1, fy)
            t.setTextColor(colors.white)
            t.setBackgroundColor(colors.blue)
            t.write(field.label)
            local buf  = bufs[i] or ""
            local show = buf
            if #show > inp_w - 1 then show = show:sub(#show - inp_w + 2) end
            local box_bg = (i == fi) and colors.lightGray or colors.gray
            t.setCursorPos(dx+1, fy+1)
            t.setBackgroundColor(box_bg)
            t.setTextColor(colors.black)
            local content = show .. ((i == fi) and "_" or " ")
            t.write(" " .. content .. string.rep(" ", math.max(0, inp_w - #content)))
        end
        local cfy = dy + 1 + (fi-1) * 3 + 1
        local cbuf = bufs[fi] or ""
        if #cbuf > inp_w - 1 then cbuf = cbuf:sub(#cbuf - inp_w + 2) end
        t.setCursorPos(dx + 1 + #cbuf + 1, cfy)
        t.setBackgroundColor(colors.lightGray)
        t.setTextColor(colors.black)
        t.setCursorBlink(true)
    end

    local focused = 1
    local bufs    = {}
    for i, f in ipairs(fields) do bufs[i] = f.default or "" end
    redraw(focused, bufs)

    local result = nil
    while true do
        local ev, p1 = os.pullEvent()
        if ev == "char" then
            bufs[focused] = bufs[focused] .. p1
            redraw(focused, bufs)
        elseif ev == "key" then
            if p1 == keys.backspace then
                if #bufs[focused] > 0 then
                    bufs[focused] = bufs[focused]:sub(1, -2)
                end
            elseif p1 == keys.enter then
                if focused < #fields then
                    focused = focused + 1
                else
                    result = bufs; break
                end
            elseif p1 == keys.tab then
                focused = (focused % #fields) + 1
            elseif p1 == keys.escape then
                break
            end
            redraw(focused, bufs)
        end
    end

    t.setCursorBlink(false)
    self.modal = false
    self:drawFrame()
    self:drawLog()
    self:drawInput()
    return result
end

return GUI
