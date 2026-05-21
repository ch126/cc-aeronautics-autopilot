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
C.SENSOR_GPS      = nil  -- gps_sensor 外设名（必须填写外设名如 "top"，nil=禁用GPS）
-- CC gps.locate() 开关：
--   false（默认）= 禁用，因为飞行器在 Sable SubLevel 上时 gps.locate()
--                  返回的是结构本地坐标而非世界坐标，会导致位置控制错乱
--   true         = 仅在电脑直接放在地面（非 SubLevel）时才能正确使用
C.USE_GPS_LOCATE  = false

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

-- navfollow 方向修正：
--   false（默认）= 直接朝导航台指向飞
--   true         = 反转180°（导航台返回的是"回家"方向时使用）
C.NAV_FOLLOW_INVERT = true
-- navfollow 到达判断：导航台 distanceToTarget() 小于此值时触发降落
-- 导航台 peripheral 没有 distanceToTarget，用角度判断：
--   abs(rel_angle) < NAV_FOLLOW_ARRIVE_DEG 持续 NAV_FOLLOW_ARRIVE_TIME 秒
-- 注意：角度接近0说明目标就在正前方很近处（已经飞过去了）
-- 若导航台有 getDistance 则优先用距离
C.NAV_FOLLOW_ARRIVE_DIST = 5.0   -- 到达距离阈值（blocks），需导航台支持 getDistance
C.NAV_FOLLOW_ARRIVE_DEG  = 25.0  -- 角度阈值（度），|rel|<25 时开始累计
C.NAV_FOLLOW_ARRIVE_TIME = 1.5   -- 累计确认时间（秒）
C.NAV_FOLLOW_MIN_TIME    = 3.0   -- 起飞后最少飞行秒数，之后才开始检测到达（防误触发）
C.NAV_RETURN_SPEED       = 0.5   -- 惯性修正速度（blocks/s），慢速往回走
C.NAV_RETURN_TIME        = 1.0   -- 惯性修正持续时间（秒），之后触发降落

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
C.PID_ATT_PITCH  = { kp=50.0, ki=0.0,  kd=10.0, dmax=60, imax=0,  omax=120 }
C.PID_ATT_ROLL   = { kp=50.0, ki=0.0,  kd=10.0, dmax=60, imax=0,  omax=120 }
C.PID_ATT_YAW    = { kp=20.0, ki=0.1,  kd=4.0,  dmax=25, imax=30, omax=90  }

-- ── 高度级联参数 ───────────────────────────────────────────────
C.ALT_POS_GAIN   = 1.0   -- alt_err -> target_climb (m/s per block)
C.ALT_MAX_CLIMB  = 1.0   -- max target climb rate (m/s)
C.ALT_VEL_GAIN   = 10.0  -- climb_err -> thr_delta (RPM per m/s)
C.ALT_MAX_DELTA  = 40    -- max throttle delta (RPM)
C.ALT_I_GAIN     = 0.01  -- altitude integrator gain
C.ALT_I_MAX      = 15    -- altitude integrator clamp (RPM)

-- ── 水平位置级联参数 ────────────────────────────────────────────
C.POS_GAIN       = 0.5   -- pos_err(blocks) -> target_vel (m/s)
C.POS_I_GAIN     = 0.02  -- 位置积分增益（消除稳态偏差）
C.POS_I_MAX      = 0.8   -- 位置积分限幅 (m/s，等效)
C.POS_MAX_VEL    = 2.0   -- max target velocity (m/s)
C.POS_DEADBAND   = 0.3   -- 位置死区 (blocks)，0.3 = 约 0.3m
C.VEL_GAIN       = 3.0   -- vel_err (m/s) -> tilt angle (deg)
C.ATT_MAX        = 15.0  -- max target pitch/roll (deg)
C.SP_SMOOTH      = 0.08  -- 悬停模式设定值平滑（goto模式自动用0.25）

-- ── 外环 PID：高度（10Hz）────────────────────────────────────────
C.PID_ALT        = { kp=5.0,  ki=0.05, kd=0.0,  imax=10, omax=50  }

-- ── 外环 PID：水平位置（10Hz，已弃用，保留备用）────────────────
C.PID_POS_X      = { kp=1.2,  ki=0.0,  kd=0.3,  imax=0,  omax=25 }
C.PID_POS_Z      = { kp=1.2,  ki=0.0,  kd=0.3,  imax=0,  omax=25 }

-- ── 手动操控（tweaked_controller）───────────────────────────
C.MANUAL_MAX_PITCH   = 40.0   -- 摇杆满偏对应的最大 pitch 角（度）
C.MANUAL_MAX_ROLL    = 40.0   -- 摇杆满偏对应的最大 roll  角（度）
C.MANUAL_YAW_RATE    = 120.0  -- 摇杆满偏对应的偏航速率（度/s）
C.MANUAL_CLIMB_RATE  = 5.0    -- 摇杆满偏对应的爬升速率（blocks/s）
-- 轴索引（tweaked_controller 官方映射，+Y 轴向下）：
--   1=左摇杆X  2=左摇杆Y  3=右摇杆X  4=右摇杆Y  5=左扳机  6=右扳机
-- 左摇杆 Y 推上=负值→取负后向上爬升；右摇杆 Y 推上=负值→取负后前倾
C.MANUAL_AXIS_CLIMB  = 2      -- 左摇杆 Y（推上=负值，取反后=爬升）
C.MANUAL_AXIS_YAW    = 1      -- 左摇杆 X（推右=正值=右偏航）
C.MANUAL_AXIS_PITCH  = 4      -- 右摇杆 Y（推上=负值，取反后=前倾）
C.MANUAL_AXIS_ROLL   = 3      -- 右摇杆 X（推右=正值=右滚转）
C.MANUAL_DEADZONE    = 0.08   -- 摇杆死区（-1..1 范围内）

return C
