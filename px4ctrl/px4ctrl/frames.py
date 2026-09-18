"""坐标系转换：控制器的 ENU/FLU ↔ PX4 的 NED/FRD。

上游 px4ctrl 把这件事交给 MAVROS（``mavros/setpoint_attitude/attitude`` 接收 ENU 姿态，
由 MAVROS 转到 PX4 的 NED）。本移植直接使用 MAVLink，因此必须自己完成转换，并且必须
**逐项可验证**——姿态符号写错会直接导致飞机翻转。

约定
----
- 世界系 ENU：x 东、y 北、z 上（ROS REP-103，也是 Isaac Sim 的约定）。
- 世界系 NED：x 北、y 东、z 下（PX4 内部）。
- 机体系 FLU：x 前、y 左、z 上（ROS）。
- 机体系 FRD：x 前、y 右、z 下（PX4）。
- 四元数一律 ``(x, y, z, w)``。

两个常量旋转
------------
``Q_ENU_TO_NED`` 把 **ENU 向量**旋转到 **NED 向量**表达：绕 (1,1,0)/√2 轴转 180°。
它对应矩阵 ``[[0,1,0],[1,0,0],[0,0,-1]]``，可用 (1,0,0)→(0,1,0) 验证。

``Q_FLU_TO_FRD`` 是机体轴系约定变更：绕 x 轴 180°。

姿态转换因此为 ``q_ned_frd = Q_ENU_TO_NED ⊗ q_enu_flu ⊗ Q_FLU_TO_FRD``。
"""

from __future__ import annotations

import math
from typing import Tuple

Quaternion = Tuple[float, float, float, float]  # (x, y, z, w)
Vector3 = Tuple[float, float, float]

_SQRT_HALF = math.sqrt(0.5)

#: ENU → NED 的向量旋转，(x, y, z, w)。
Q_ENU_TO_NED: Quaternion = (_SQRT_HALF, _SQRT_HALF, 0.0, 0.0)

#: FLU → FRD 的机体轴系变更，(x, y, z, w)。
Q_FLU_TO_FRD: Quaternion = (1.0, 0.0, 0.0, 0.0)


def quat_mul(a: Quaternion, b: Quaternion) -> Quaternion:
    """四元数乘法 ``a ⊗ b``（先施加 b，再施加 a）。"""

    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q: Quaternion) -> Quaternion:
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_normalize(q: Quaternion) -> Quaternion:
    """归一化；零四元数视为单位四元数，避免除零。"""

    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_rotate(q: Quaternion, v: Vector3) -> Vector3:
    """用四元数旋转向量：``v' = q ⊗ v ⊗ q*``。"""

    q = quat_normalize(q)
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * (q_vec × v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    # v' = v + w*t + q_vec × t
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def enu_to_ned_position(p: Vector3) -> Vector3:
    """位置/速度：ENU → NED（North=y_e, East=x_e, Down=-z_e）。"""

    east, north, up = p
    return (north, east, -up)


def ned_to_enu_position(p: Vector3) -> Vector3:
    """位置/速度：NED → ENU（该映射自逆）。"""

    north, east, down = p
    return (east, north, -down)


#: WGS-84 椭球参数，用于地理坐标 ↔ 局部 ENU 的切线平面换算。
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
    origin_alt_m: float,
    earth_radius: float | None = None,
) -> Vector3:
    """地理坐标 → **共享**局部 ENU（以 ``origin_*`` 为原点）。

    为什么需要它
    ------------
    各机 PX4 EKF 的 ``LOCAL_POSITION_NED`` 原点由**各自的**起飞点建立，因此两机的
    local 坐标不在同一系里，直接相减得不到真实相对位置（实测同高度的两机局部 z
    相差约 1 cm，且差值随各自起飞点变化）。而两机的 GPS 都相对**同一个**世界地理
    原点，所以换算到以该原点为基准的 ENU 后，位置才可比。

    地球模型（必须与 GPS 的产生方一致）
    -----------------------------------
    - ``earth_radius is None``：使用 **WGS-84** 椭球曲率半径（子午圈 ``M``、卯酉圈
      ``N``）。这是**真机**的正确选择。
    - ``earth_radius`` 给定时：使用该半径的**球面**大圆模型。

    为什么需要后者：仿真里 Pegasus 用 ``EARTH_RADIUS = 6353000.0`` 的球面重投影
    （``geo_mag_utils.reprojection``），与椭球并不一致。在纬度 38.74° 处
    ``N/R ≈ 1.0052``，即东向有约 0.52% 的尺度差，5 m 基线就是约 2.6 cm 的**系统性**
    偏差。让反解与产生方用同一模型，才能把这段人为误差消掉，剩下的才是真实的
    GPS 噪声与 EKF 估计误差。

    精度说明
    --------
    采用切线平面近似：在原点纬度处把经纬增量线性化为北/东位移。两机相距仅数米时，
    跨切线面的旋转约 ``d / R ≈ 1e-6 rad``，可忽略。

    姿态与速度**不需要**该换算：本地 NED 与共享 ENU 只差坐标轴定义（同为"上=上"），
    没有原点偏移问题。只有位置受各机 EKF 原点影响。
    """

    if earth_radius is not None:
        # 球面模型：与 Pegasus 的重投影严格互逆。
        d_lat = math.radians(lat_deg - origin_lat_deg)
        d_lon = math.radians(lon_deg - origin_lon_deg)
        north = d_lat * earth_radius
        east = d_lon * earth_radius * math.cos(math.radians(origin_lat_deg))
        up = alt_m - origin_alt_m
        return (east, north, up)

    lat0 = math.radians(origin_lat_deg)
    sin_lat0 = math.sin(lat0)
    denom = 1.0 - WGS84_E2 * sin_lat0 * sin_lat0

    # N：卯酉圈曲率半径；M：子午圈曲率半径。
    radius_meridional = WGS84_A * (1.0 - WGS84_E2) / (denom ** 1.5)
    radius_normal = WGS84_A / math.sqrt(denom)

    d_lat = math.radians(lat_deg - origin_lat_deg)
    d_lon = math.radians(lon_deg - origin_lon_deg)

    north = d_lat * radius_meridional
    east = d_lon * radius_normal * math.cos(lat0)
    up = alt_m - origin_alt_m
    return (east, north, up)


def enu_flu_to_ned_frd(q: Quaternion) -> Quaternion:
    """姿态：ENU/FLU → NED/FRD。"""

    return quat_normalize(quat_mul(quat_mul(Q_ENU_TO_NED, q), Q_FLU_TO_FRD))


def ned_frd_to_enu_flu(q: Quaternion) -> Quaternion:
    """姿态：NED/FRD → ENU/FLU（:func:`enu_flu_to_ned_frd` 的逆）。"""

    return quat_normalize(
        quat_mul(quat_mul(quat_conjugate(Q_ENU_TO_NED), q), quat_conjugate(Q_FLU_TO_FRD))
    )


def self_check() -> None:
    """对两个常量旋转做解析可验证的断言。

    这些断言是姿态链路的"红线测试"：一旦有人改错符号，这里立刻失败，而不是等到
    飞机在仿真里翻转才发现。
    """

    # ENU 的"东"(1,0,0) 在 NED 中应为"北"→(0,1,0)。
    east_in_ned = quat_rotate(Q_ENU_TO_NED, (1.0, 0.0, 0.0))
    assert _close(east_in_ned, (0.0, 1.0, 0.0)), east_in_ned

    # ENU 的"上"(0,0,1) 在 NED 中应为"下"→(0,0,-1)。
    up_in_ned = quat_rotate(Q_ENU_TO_NED, (0.0, 0.0, 1.0))
    assert _close(up_in_ned, (0.0, 0.0, -1.0)), up_in_ned

    # 水平且机头朝东：ENU/FLU 姿态为单位四元数。
    # NED/FRD 下机体前向应指向"东"→ NED (0,1,0)。
    q_ned_frd = enu_flu_to_ned_frd((0.0, 0.0, 0.0, 1.0))
    forward_in_ned = quat_rotate(q_ned_frd, (1.0, 0.0, 0.0))
    assert _close(forward_in_ned, (0.0, 1.0, 0.0)), forward_in_ned

    # 机体"下"应指向 NED 的下。
    down_in_ned = quat_rotate(q_ned_frd, (0.0, 0.0, 1.0))
    assert _close(down_in_ned, (0.0, 0.0, 1.0)), down_in_ned

    # 位置映射自逆。
    assert _close(ned_to_enu_position(enu_to_ned_position((1.0, 2.0, 3.0))), (1.0, 2.0, 3.0))

    # 共享系换算：原点本身必须映射到零。
    origin = (38.736832, -9.137977, 90.0)
    assert _close(geodetic_to_enu(*origin, *origin), (0.0, 0.0, 0.0)), "原点不为零"

    # 1e-3 度量级的经纬增量应得到符合 WGS-84 曲率半径的米级位移。
    # 在纬度 38.74°：北向 ≈ M·Δlat ≈ 110.9 m，东向 ≈ N·cos(lat)·Δlon ≈ 86.9 m。
    east, north, up = geodetic_to_enu(
        origin[0] + 1e-3, origin[1] + 1e-3, origin[2] + 5.0, *origin
    )
    assert 110.0 < north < 112.0, north
    assert 86.0 < east < 88.0, east
    assert abs(up - 5.0) < 1e-9, up

    # 姿态往返一致。
    original = quat_normalize((0.1, -0.2, 0.3, 0.9))
    assert _close(
        ned_frd_to_enu_flu(enu_flu_to_ned_frd(original)), original, tol=1e-9
    ), "姿态往返不一致"


def _close(a: Vector3 | Quaternion, b: Vector3 | Quaternion, tol: float = 1e-6) -> bool:
    return all(abs(x - y) <= tol for x, y in zip(a, b))


if __name__ == "__main__":
    self_check()
    print("frames self_check: OK")
