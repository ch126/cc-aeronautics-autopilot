-- =============================================================
--  startup.lua
  -- /startup.lua
  -- CC:Tweaked
-- =============================================================

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
