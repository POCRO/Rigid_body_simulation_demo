import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 读取 CSV 文件
# data = np.loadtxt("trajectory.csv", delimiter=",")
data = np.loadtxt("/home/yy/ros2_ws/trajectory.csv", delimiter=",")
data_free_drop = np.loadtxt("/home/yy/ros2_ws/src/rigid_body_simulation/log/trajectory_free_drop.csv", delimiter=",")
data_with_lift = np.loadtxt("/home/yy/ros2_ws/src/rigid_body_simulation/log/trajectory_with_lift.csv", delimiter=",")
data_zero_lift = np.loadtxt("/home/yy/ros2_ws/src/rigid_body_simulation/log/trajectory_zero_lift.csv", delimiter=",")

# 提取 x, y, z 坐标
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

x_free_drop = data_free_drop[:, 0]
y_free_drop = data_free_drop[:, 1]
z_free_drop = data_free_drop[:, 2]

x_with_lift = data_with_lift[:, 0]
y_with_lift = data_with_lift[:, 1]
z_with_lift = data_with_lift[:, 2]

x_zero_lift = data_zero_lift[:, 0]
y_zero_lift = data_zero_lift[:, 1]
z_zero_lift = data_zero_lift[:, 2]

# 创建 3D 图
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制轨迹
# ax.plot(x, y, z, label="Reentry Trajectory", color='b')
ax.plot(x_free_drop, y_free_drop, z_free_drop, label="freedrop Trajectory", color='r')
ax.plot(x_with_lift, y_with_lift, z_with_lift, label="lift on Trajectory", color='g')
ax.plot(x_zero_lift, y_zero_lift, z_zero_lift, label="no lift Trajectory", color='b')


ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_zlabel("Z Position (m)")
ax.set_title("Reentry Trajectory Visualization")
ax.legend()

# 显示图形
plt.show()
