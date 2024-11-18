from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 获取 URDF 文件的绝对路径
    package_dir = get_package_share_directory('rigid_body_simulation')
    urdf_path = os.path.join(package_dir, 'urdf', 'rigid_body.urdf')
    # 构建参数文件路径
    
    params_file = os.path.join(package_dir, 'config', 'simulation_params.yaml')

    # 检查 URDF 文件是否存在
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")

    return LaunchDescription([
        # 加载 URDF 文件
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": open(urdf_path).read()}]
        ),
        # 刚体仿真节点
        Node(
            package="rigid_body_simulation",
            executable="rigid_body_simulation_node",
            name="rigid_body_node",
            output="screen",
            parameters=[params_file]
        )
    ])
