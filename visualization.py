
"""
visualization.py
关节驱动力矩可视化
"""

import matplotlib.pyplot as plt


def plot_joint_torques(time, torques):
    """
    绘制关节力矩曲线

    time: 时间数组，shape (N,)
    torques: 关节力矩数组，shape (N, 2)
    """

    plt.figure(figsize=(10, 5))

    # 关节1
    plt.plot(
        time,
        torques[:, 0],
        label="Joint 1 Torque",
        linewidth=2
    )

    # 关节2
    plt.plot(
        time,
        torques[:, 1],
        label="Joint 2 Torque",
        linewidth=2
    )

    plt.xlabel("Time (s)")
    plt.ylabel("Torque (N*m)")

    plt.title("2-DOF Robot Arm Joint Torques")

    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.show()
