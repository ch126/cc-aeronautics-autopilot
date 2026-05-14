-- =============================================================
--  quad/ping.lua
--  Test rednet connection to all slave computers
--  Run on MASTER computer: /quad/ping
-- =============================================================
term.clear()
term.setCursorPos(1,1)

local C = dofile("/quad/config.lua")
local PROTOCOL = C.MOTOR_PROTOCOL or "quad_motor"

print("=== Quad Slave Ping Test ===")
print("Protocol: " .. PROTOCOL)
print("")

-- open rednet
peripheral.find("modem", rednet.open)

local slave_map = {
    { label="FL", id=C.SLAVE_FL },
    { label="FR", id=C.SLAVE_FR },
    { label="BR", id=C.SLAVE_BR },
    { label="BL", id=C.SLAVE_BL },
}

-- send test RPM to each slave and wait for ACK
for _, s in ipairs(slave_map) do
    if s.id then
        io.write(string.format("  Pinging %s (ID=%d) ... ", s.label, s.id))
        rednet.send(s.id, { idx=0, rpm=0 }, PROTOCOL)
        local sender, reply = rednet.receive(PROTOCOL, 2)  -- 2s timeout
        if sender == s.id and type(reply) == "table" and reply.ack then
            print("OK (replied from #" .. sender .. ")")
        elseif sender then
            print("UNKNOWN REPLY from #" .. sender)
        else
            print("TIMEOUT - no response")
        end
    else
        print(string.format("  %s: not configured (nil)", s.label))
    end
end

print("")
print("Done. If TIMEOUT: check slave is running /quad/slave")
print("and both computers have Ender Modem attached.")
