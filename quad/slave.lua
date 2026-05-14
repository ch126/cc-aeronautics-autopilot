-- =============================================================
--  quad/slave.lua
--  Motor slave computer - controls ONE speed controller
--  Run this on each of the 4 motor computers.
--
--  Setup:
--    1. Place this computer next to a Create_RotationSpeedController
--    2. Connect with wired modem + cable
--    3. Run: /quad/slave
--    4. Note the computer ID shown, enter it in master's config.lua
--       as C.SLAVE_FL / SLAVE_FR / SLAVE_BR / SLAVE_BL
-- =============================================================

term.clear()
term.setCursorPos(1,1)

local PROTOCOL = "quad_motor"

-- open rednet
peripheral.find("modem", rednet.open)

-- find speed controller
local gear = peripheral.find("Create_RotationSpeedController")
if not gear then
    printError("ERROR: Create_RotationSpeedController not found!")
    printError("Check wired modem is connected and active.")
    return
end

local my_id = os.getComputerID()
print("=== Quad Motor Slave ===")
print("Computer ID: " .. my_id)
print("Tell master: C.SLAVE_?? = " .. my_id)
print("Protocol: " .. PROTOCOL)
print("Speed controller: OK")
print("")
print("Listening for RPM commands ...")
print("(Ctrl+T to stop)")

gear.setTargetSpeed(0)

-- main loop
while true do
    local sender, msg = rednet.receive(PROTOCOL)
    if type(msg) == "table" and type(msg.rpm) == "number" then
        local rpm = math.floor(msg.rpm + 0.5)
        gear.setTargetSpeed(rpm)
        -- show on screen
        term.setCursorPos(1, 9)
        term.clearLine()
        term.write(string.format("From #%d  rpm=%d      ", sender, rpm))
    end
end
