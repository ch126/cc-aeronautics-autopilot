-- =============================================================
--  startup.lua
  -- /startup.lua
  -- CC:Tweaked
-- =============================================================

-- dofile 缓存：防止同一文件被重复执行（模拟 require 的缓存行为）
_LOADED = {}
local _real_dofile = dofile
function dofile(path)
    if _LOADED[path] then return _LOADED[path] end
    local result = _real_dofile(path)
    _LOADED[path] = result or true
    return result
end

  -- 1
os.sleep(1)

  -- Logo
term.clear()
term.setCursorPos(1, 1)
term.setTextColor(colors.cyan)
print("+----------------------------------------------+")
print("||  Create:Aeronautics Autopilot  Booting...   ||")
print("+----------------------------------------------+")
term.setTextColor(colors.white)


shell.setDir("/")
dofile("/autopilot/main.lua")
