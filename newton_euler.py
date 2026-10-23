
"""
newton_euler.py
两自由度机械臂递推牛顿-欧拉逆动力学
"""

import numpy as np


def rnea_2r(q, dq, ddq, params):
    """
    Recursive Newton-Euler Algorithm

    Parameters
    ----------
    q : array_like, shape (2,)
        关节角度，单位 rad
    dq : array_like, shape (2,)
        关节角速度，单位 rad/s
    ddq : array_like, shape (2,)
        关节角加速度，单位 rad/s^2
    params : dict
        机械臂物理参数

    Returns
    -------
    tau : ndarray, shape (2,)
        关节驱动力矩，单位 N*m
    """

    q = np.asarray(q, dtype=float)
    dq = np.asarray(dq, dtype=float)
    ddq = np.asarray(ddq, dtype=float)

    if any(x.shape != (2,) for x in (q, dq, ddq)):
        raise ValueError("q、dq、ddq必须是长度为2的向量")

    q1, q2 = q
    dq1, dq2 = dq
    ddq1, ddq2 = ddq

    # 机械臂参数
    L1 = params["L1"]
    lc1 = params["lc1"]
    lc2 = params["lc2"]

    m1 = params["m1"]
    m2 = params["m2"]

    I1 = params["I1"]
    I2 = params["I2"]

    g = params["g"]

    # Z轴单位向量
    z = np.array([0.0, 0.0, 1.0])

    # 重力加速度向量
    gravity = np.array([0.0, -g, 0.0])

    # =====================================
    # 1. 位置向量
    # =====================================

    # 关节1到关节2
    r12 = L1 * np.array([
        np.cos(q1),
        np.sin(q1),
        0.0
    ])

    # 关节1到连杆1质心
    rc1 = lc1 * np.array([
        np.cos(q1),
        np.sin(q1),
        0.0
    ])

    # 关节2到连杆2质心
    rc2 = lc2 * np.array([
        np.cos(q1 + q2),
        np.sin(q1 + q2),
        0.0
    ])

    # =====================================
    # 2. 向外递推：运动学
    # =====================================

    # 基座虚拟加速度（包含重力）
    a0 = -gravity

    # ---------- 连杆1 ----------

    # 角速度
    w1 = dq1 * z

    # 角加速度
    alpha1 = ddq1 * z

    # 关节1的线加速度
    aj1 = a0

    # 连杆1质心加速度
    ac1 = (
        aj1
        + np.cross(alpha1, rc1)
        + np.cross(w1, np.cross(w1, rc1))
    )

    # ---------- 连杆2 ----------

    # 角速度
    w2 = w1 + dq2 * z

    # 角加速度
    alpha2 = alpha1 + ddq2 * z

    # 关节2的线加速度
    aj2 = (
        aj1
        + np.cross(alpha1, r12)
        + np.cross(w1, np.cross(w1, r12))
    )

    # 连杆2质心加速度
    ac2 = (
        aj2
        + np.cross(alpha2, rc2)
        + np.cross(w2, np.cross(w2, rc2))
    )

    # =====================================
    # 3. 向内递推：动力学
    # =====================================

    # ---------- 连杆2 ----------

    # 惯性力
    F2 = m2 * ac2

    # 惯性力矩
    N2 = I2 * alpha2

    # 关节2的合力
    f2 = F2

    # 关节2的合力矩
    n2 = N2 + np.cross(rc2, F2)

    # 关节2驱动力矩
    tau2 = np.dot(n2, z)

    # ---------- 连杆1 ----------

    # 惯性力
    F1 = m1 * ac1

    # 惯性力矩
    N1 = I1 * alpha1

    # 关节1的合力
    f1 = F1 + f2

    # 关节1的合力矩
    n1 = (
        N1
        + np.cross(rc1, F1)
        + n2
        + np.cross(r12, f2)
    )

    # 关节1驱动力矩
    tau1 = np.dot(n1, z)

    return np.array([tau1, tau2])
