
"""
main.py
两自由度机械臂牛顿-欧拉动力学主程序
"""

import numpy as np

from robot_parameters import ROBOT_PARAMS
from newton_euler import rnea_2r
from visualization import plot_joint_torques


def static_test():
    """实验1：静止状态力矩测试"""

    print("===== 静止状态测试 =====")

    q = np.array([0.0, 0.0])
    dq = np.array([0.0, 0.0])
    ddq = np.array([0.0, 0.0])

    tau = rnea_2r(q, dq, ddq, ROBOT_PARAMS)

    print(f"Joint 1 Torque: {tau[0]:.4f} N*m")
    print(f"Joint 2 Torque: {tau[1]:.4f} N*m")


def trajectory_test():
    """实验2：关节轨迹动力学计算"""

    print("\n===== 连续运动测试 =====")

    # 时间范围
    time = np.linspace(0, 10, 1001)

    # 保存两个关节的驱动力矩
    torques = np.zeros((len(time), 2))

    for i, t in enumerate(time):

        # 定义关节运动轨迹
        q1 = 0.4 * np.sin(t)
        q2 = 0.3 * np.sin(1.5 * t)

        # 关节角速度（一阶导数）
        dq1 = 0.4 * np.cos(t)
        dq2 = 0.45 * np.cos(1.5 * t)

        # 关节角加速度（二阶导数）
        ddq1 = -0.4 * np.sin(t)
        ddq2 = -0.675 * np.sin(1.5 * t)

        # 构造关节状态向量
        q = np.array([q1, q2])
        dq = np.array([dq1, dq2])
        ddq = np.array([ddq1, ddq2])

        # 调用牛顿-欧拉算法
        tau = rnea_2r(q, dq, ddq, ROBOT_PARAMS)

        # 保存结果
        torques[i] = tau

    print("轨迹计算完成")

    # 绘制驱动力矩曲线
    plot_joint_torques(time, torques)


def main():

    static_test()

    trajectory_test()


if __name__ == "__main__":
    main()
