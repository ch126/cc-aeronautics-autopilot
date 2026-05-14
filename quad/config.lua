-- =============================================================
--  quad/config.lua
--  四旋翼飞控配置
--
--  硬件假设：
--    Create 机械动力「转速控制器」(Rotation Speed Controller)
--    通过有线调制解调器连接到 CC:Tweaked 高级电脑
--
--    外设 API（每个转速控制器）:
--      peripheral.wrap(name).setTargetSpeed(rpm)   设定目标转速
--      peripheral.wrap(name).getSpeed()            读取实际转速（可选）
--
--  X型四旋翼俯视布局：
--
--        前(North)
--      M1(↺)  M2(↻)
--         \  /
--          \/
--          /\
--         /  \
--      M4(↻)  M3(↺)
--        后(South)
--
--  M1=左前CCW  M2=右前CW  M3=右后CCW  M4=左后CW
--  RPM符号：正值=逆时针(CCW)，负值=顺时针(CW)
--  偏航：M1+M3加速(CCW) → 机体右偏航(CW)
-- =============================================================

local C = {}

-- ── 转速控制器外设名称 ─────────────────────────────────────────
-- 设为 nil 时按顺序自动分配找到的 "speed_controller" 外设
C.MOTOR_FL = nil   -- M1 左前  (逆时针)
C.MOTOR_FR = nil   -- M2 右前  (顺时针)
C.MOTOR_BR = nil   -- M3 右后  (逆时针)
C.MOTOR_BL = nil   -- M4 左后  (顺时针)

-- 转速控制器外设类型名（根据实际 mod 填写）
C.MOTOR_TYPE = "Create_RotationSpeedController"

-- ── 转速参数 ───────────────────────────────────────────────────
C.RPM_MAX      = 256    -- 最大转速 (RPM)，不超过转速控制器上限
C.RPM_MIN      = 0      -- 最小转速（悬停需要一定转速，可设基础值）
C.RPM_HOVER    = 64     -- 悬停基础转速（需根据机体重量标定）
C.RPM_IDLE     = 0      -- 解锁后待机转速

-- ── 控制频率 ───────────────────────────────────────────────────
C.CTRL_HZ      = 50     -- 内环（姿态）控制频率 Hz
C.NAV_HZ       = 10     -- 外环（位置）控制频率 Hz
C.CTRL_DT      = 1 / 50 -- 内环 dt
C.NAV_DT       = 1 / 10 -- 外环 dt

-- ── 传感器外设名称（nil=自动扫描）─────────────────────────────
C.SENSOR_GIMBAL   = nil  -- gimbal_sensor  -> getAngles() {pitch,roll,yaw}
C.SENSOR_ALTITUDE = nil  -- altitude_sensor -> getHeight()
C.SENSOR_VELOCITY = nil  -- velocity_sensor -> getVelocity()

-- ── 飞行参数 ───────────────────────────────────────────────────
C.MAX_TILT       = 25.0  -- max tilt angle (deg)
C.MAX_CLIMB      = 4.0   -- max climb rate (m/s)
C.MAX_SPEED      = 6.0   -- max horizontal speed (m/s)
C.ARRIVAL_RADIUS = 2.0   -- arrival radius (blocks)
C.ALT_THRESHOLD  = 0.5   -- altitude deadband (blocks)
C.ATT_MAX        = 25.0  -- max target pitch/roll from position loop (deg)
C.RATE_MAX       = 180.0 -- max target rate from attitude loop (deg/s)

-- ── 内环 PID：姿态角速度（最内层，50Hz）─────────────────────────
-- 输入：目标角速度(°/s) - 实际角速度(°/s)，输出：RPM差量
C.PID_RATE_PITCH = { kp=1.2,  ki=0.02, kd=0.05, imax=20, omax=60 }
C.PID_RATE_ROLL  = { kp=1.2,  ki=0.02, kd=0.05, imax=20, omax=60 }
C.PID_RATE_YAW   = { kp=1.0,  ki=0.01, kd=0.02, imax=15, omax=40 }

-- ── 中环 PID：姿态角（50Hz）──────────────────────────────────────
-- 输入：目标角度(deg) - 实际角度(deg)，输出：目标角速度(°/s)
C.PID_ATT_PITCH  = { kp=4.5,  ki=0.0,  kd=0.0,  imax=0,  omax=180 }
C.PID_ATT_ROLL   = { kp=4.5,  ki=0.0,  kd=0.0,  imax=0,  omax=180 }
C.PID_ATT_YAW    = { kp=3.0,  ki=0.05, kd=0.1,  imax=30, omax=120 }

-- ── 外环 PID：高度（10Hz）────────────────────────────────────────
-- 输入：目标高度(blocks) - 实际高度，输出：油门增量(RPM)
C.PID_ALT        = { kp=8.0,  ki=0.5,  kd=2.0,  imax=30, omax=80 }

-- ── 外环 PID：水平位置（10Hz，需GPS）─────────────────────────────
-- 输出：目标倾斜角(deg)
C.PID_POS_X      = { kp=1.2,  ki=0.0,  kd=0.3,  imax=0,  omax=25 }
C.PID_POS_Z      = { kp=1.2,  ki=0.0,  kd=0.3,  imax=0,  omax=25 }

return C
