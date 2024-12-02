# rigid_body_simulation_demo


一个基于 C++ 和 ROS 2 的多体动力学仿真demo


> 开发环境：Ubuntu22.04 、ROS2 humble
> 

# 0.1 版本-2024-11-18

## 1. 四阶 Runge-Kuta法

## 2. 从simulation_params.yaml中动态读取初值条件、刚体属性
```
rigid_body_node:
  ros__parameters:
    mass: 1.0 # 质量（kg）
    friction_coefficient: 0.1 # 摩擦系数 
    lift_coefficient: 0.001 # 升力系数 
    time_step: 0.01  # 时间步长 (s) 
    inertia: [10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 10.0] 
    external_force: [10.0, 0.0, 1.0] # 外力 
    external_torque: [0.0001, 0.0, 0.0] # 外力矩 
    gravity: [0.0, 0.0, -2.0] # 重力 
    initial_position: [0.0, 0.0, 1000.0]  # 初始位置 (x, y, z) 
    initial_velocity: [5.0, 0.0, 0.0]  # 初始速度 (vx, vy, vz) 
    # initial_orientation: [1.0, 0.0, 0.0, 0.0]  # 初始四元数 (w, x, y, z) 

```
## 3. 添加摩擦力、升力、重力、给定恒外力，无科氏力

## 4. TF树发布刚体位姿以在rviz2中实现可视化

## 5. 未完成：各力方向可视化，轨迹可视化,RigidBodyNode类太臃肿，后续开发应该把各部分模块化、


# 0.2 版本-2024-11-25 

## 修改气动力形式，添加readme

# 0.3 版本-2024-12-1

## 尝试tudat c++库，失败告终
<<<<<<< HEAD


# 0.4 版本-2024-12-2

## 添加大气场，重力场因素，添加轨迹绘制，跑了三组测试样例：自由落体，无升力再入，有升力再入
=======
>>>>>>> 806fd1aa51824a0bc12e91d9c03df0b4f79c2847
