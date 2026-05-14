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

-- ── 分布式从机设置 ─────────────────────────────────────────────
-- 如果用 4 台从机电脑各自控制一个转速控制器，填写各从机的电脑ID
-- 在从机上运行 /quad/slave，然后用 `id` 命令查看从机ID
-- 设为 nil 表示主机直连模式（不使用从机）
C.SLAVE_FL = 25   -- M1 左前从机电脑ID
C.SLAVE_FR = 23   -- M2 右前从机电脑ID
C.SLAVE_BR = 24   -- M3 右后从机电脑ID
C.SLAVE_BL = 26   -- M4 左后从机电脑ID
C.MOTOR_PROTOCOL = "quad_motor"  -- rednet 通信协议名

-- ── 转速参数 ───────────────────────────────────────────────────
C.RPM_MAX      = 256    -- 最大转速 (RPM)
C.RPM_MIN      = 0      -- 最小转速
C.RPM_HOVER    = 80     -- 悬停基础转速（需根据机体重量标定）
C.RPM_IDLE     = 0      -- 解锁后待机转速

-- ── 控制频率 ───────────────────────────────────────────────────
C.CTRL_HZ      = 20     -- 内环（姿态）控制频率 Hz（CC服务器tick上限~20Hz）
C.NAV_HZ       = 20     -- 外环（位置）控制频率 Hz（与内环合并，减少延迟）
C.CTRL_DT      = 1 / 20 -- 内环 dt
C.NAV_DT       = 1 / 20 -- 外环 dt

-- ── 传感器外设名称（nil=自动扫描）─────────────────────────
C.SENSOR_GIMBAL   = nil  -- gimbal_sensor  -> getAngles() {pitch,roll,yaw}
C.SENSOR_ALTITUDE = nil  -- altitude_sensor -> getHeight()
-- 两个速度传感器：一个朝 X 轴，一个朝 Z 轴（nil=自动找第一个）
C.SENSOR_VEL_X    = nil  -- velocity_sensor 朝 X 轴 -> getVelocity() 返回标量 m/s
C.SENSOR_VEL_Z    = nil  -- velocity_sensor 朝 Z 轴 -> getVelocity() 返回标量 m/s
C.SENSOR_NAV      = nil  -- navigation_table -> getRelativeAngle() 导航台修正yaw

-- ── 导航台 Heading 修正 ────────────────────────────────────────
-- 将罗盘/磁铁放在地面，飞机上装导航台
-- NAV_BEACON_BEARING：从起飞点看导航台的绝对方位角（度，北=0，东=90）
-- 设为 nil 则禁用导航台修正
C.NAV_BEACON_BEARING = nil   -- 例如：0 = 磁铁在正北方
-- 互补滤波系数：gimbal占比（越大=越信gimbal，越小=越信导航台）
C.NAV_YAW_ALPHA      = 0.98  -- 0.98 = 慢速漂移修正，快速旋转靠gimbal

-- 导航台位置修正：信标相对起飞点的坐标（blocks，北=+X，东=+Z）
-- 填写后，导航台方位角会修正航位推算的位置误差
-- 设为 nil 则只用方位角修正 yaw，不修正位置
C.NAV_BEACON_X   = nil   -- 信标相对起飞点的 X 坐标 (blocks)
C.NAV_BEACON_Z   = nil   -- 信标相对起飞点的 Z 坐标 (blocks)
C.NAV_POS_ALPHA  = 0.3   -- 导航台位置收敛速度（0.3 = 快速跟踪，纯导航台模式用大值）

-- ── 飞行参数 ───────────────────────────────────────────────────
C.MAX_TILT       = 25.0  -- max tilt angle (deg)
C.MAX_CLIMB      = 4.0   -- max climb rate (m/s)
C.MAX_SPEED      = 6.0   -- max horizontal speed (m/s)
C.ARRIVAL_RADIUS = 2.0   -- arrival radius (blocks)
C.ALT_THRESHOLD  = 0.5   -- altitude deadband (blocks)
C.ATT_MAX        = 15.0  -- max target pitch/roll from position loop (deg)
C.RATE_MAX       = 180.0 -- max target rate from attitude loop (deg/s)

-- ── 内环 PID：姿态角速度（最内层，20Hz）─────────────────────────
C.PID_RATE_PITCH = { kp=0.12, ki=0.0,  kd=0.0,  imax=0,  omax=12 }
C.PID_RATE_ROLL  = { kp=0.12, ki=0.0,  kd=0.0,  imax=0,  omax=12 }
C.PID_RATE_YAW   = { kp=0.25, ki=0.0,  kd=0.0,  imax=0,  omax=10 }

-- ── 中环 PID：姿态角（20Hz）──────────────────────────────────────
-- 直接输出除以 RATE_MAX 得到 -1..1，不再走 rate loop
-- kd 作用于测量值（D-on-measurement），等效于角速度阻尼
-- dmax 限制D项最大贡献，防止噪声脉冲引起大幅转速差
C.PID_ATT_PITCH  = { kp=20.0, ki=0.0,  kd=6.0,  dmax=45, imax=0,  omax=120 }
C.PID_ATT_ROLL   = { kp=20.0, ki=0.0,  kd=6.0,  dmax=45, imax=0,  omax=120 }
C.PID_ATT_YAW    = { kp=15.0, ki=0.1,  kd=2.5,  dmax=25, imax=30, omax=90  }

-- ── 高度级联参数 ───────────────────────────────────────────────
C.ALT_POS_GAIN   = 1.0   -- alt_err -> target_climb (m/s per block)
C.ALT_MAX_CLIMB  = 1.0   -- max target climb rate (m/s)
C.ALT_VEL_GAIN   = 10.0  -- climb_err -> thr_delta (RPM per m/s)
C.ALT_MAX_DELTA  = 40    -- max throttle delta (RPM)
C.ALT_I_GAIN     = 0.01  -- altitude integrator gain
C.ALT_I_MAX      = 15    -- altitude integrator clamp (RPM)

-- ── 水平位置级联参数 ────────────────────────────────────────────
C.POS_GAIN       = 0.5   -- pos_err -> target_vel (m/s per block)
C.POS_MAX_VEL    = 2.0   -- max target velocity (m/s)
C.POS_DEADBAND   = 0.3   -- 位置死区 (blocks)，0.3 = 约 0.3m
C.VEL_GAIN       = 3.0   -- vel_err -> tilt angle (deg per m/s)
C.ATT_MAX        = 15.0  -- max target pitch/roll (deg)
C.SP_SMOOTH      = 0.08  -- 悬停模式设定值平滑（goto模式自动用0.25）

-- ── 外环 PID：高度（10Hz）────────────────────────────────────────
C.PID_ALT        = { kp=5.0,  ki=0.05, kd=0.0,  imax=10, omax=50  }

-- ── 外环 PID：水平位置（10Hz，已弃用，保留备用）────────────────
C.PID_POS_X      = { kp=1.2,  ki=0.0,  kd=0.3,  imax=0,  omax=25 }
C.PID_POS_Z      = { kp=1.2,  ki=0.0,  kd=0.3,  imax=0,  omax=25 }

return C
