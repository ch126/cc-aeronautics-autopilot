-- =============================================================
--  autopilot/vec3.lua
--  轻量三维向量库
-- =============================================================

local Vec3 = {}
Vec3.__index = Vec3

function Vec3.new(x, y, z)
    return setmetatable({ x = x or 0, y = y or 0, z = z or 0 }, Vec3)
end

function Vec3:clone()
    return Vec3.new(self.x, self.y, self.z)
end

function Vec3.__add(a, b)  return Vec3.new(a.x+b.x, a.y+b.y, a.z+b.z) end
function Vec3.__sub(a, b)  return Vec3.new(a.x-b.x, a.y-b.y, a.z-b.z) end
function Vec3.__unm(a)     return Vec3.new(-a.x, -a.y, -a.z) end
function Vec3.__mul(a, s)
    if type(a) == "number" then return Vec3.new(a*s.x, a*s.y, a*s.z) end
    return Vec3.new(a.x*s, a.y*s, a.z*s)
end
function Vec3.__div(a, s)  return Vec3.new(a.x/s, a.y/s, a.z/s) end
function Vec3.__eq(a, b)   return a.x==b.x and a.y==b.y and a.z==b.z end
function Vec3:__tostring() return string.format("(%.2f, %.2f, %.2f)", self.x, self.y, self.z) end

function Vec3:lengthSq()   return self.x^2 + self.y^2 + self.z^2 end
function Vec3:length()     return math.sqrt(self:lengthSq()) end

function Vec3:normalize()
    local len = self:length()
    if len < 1e-9 then return Vec3.new(0,0,0) end
    return self / len
end

function Vec3:dot(b)   return self.x*b.x + self.y*b.y + self.z*b.z end
function Vec3:cross(b)
    return Vec3.new(
        self.y*b.z - self.z*b.y,
        self.z*b.x - self.x*b.z,
        self.x*b.y - self.y*b.x
    )
end

-- 水平距离（忽略 Y）
function Vec3:horzDist(b)
    local dx = self.x - b.x
    local dz = self.z - b.z
    return math.sqrt(dx*dx + dz*dz)
end

-- 水平方向角（相对 -Z 轴，顺时针为正，与 MC Yaw 一致）
function Vec3:yawTo(b)
    local dx = b.x - self.x
    local dz = b.z - self.z
    -- math.atan2(x, -z) → MC yaw (0=南/-Z, 90=西/-X, ±180=北/+Z, -90=东/+X)
    local yaw = math.deg(math.atan(dx, -dz))
    return yaw
end

-- 规范化角度到 [-180, 180]
function Vec3.normalizeAngle(a)
    a = a % 360
    if a > 180 then a = a - 360 end
    return a
end

-- 线性插值
function Vec3.lerp(a, b, t)
    return Vec3.new(
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t
    )
end

return Vec3
