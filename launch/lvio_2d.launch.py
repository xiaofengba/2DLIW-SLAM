import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 lvio_2d 包的安装共享目录
    lvio_2d_share_dir = get_package_share_directory('lvio_2d')

    # 定义配置文件的绝对路径
    config_file = os.path.join(lvio_2d_share_dir, 'config', 'ros2_params.yaml')
    rviz_config_file = os.path.join(lvio_2d_share_dir, 'launch', 'display.rviz')

    # 定义主节点 (lvio_2d_node)
    lvio_2d_node = Node(
        package='lvio_2d',
        executable='lvio_2d_node',   # ROS 2 中使用 executable 替代 type
        name='lvio_2d_node',
        output='screen',
        parameters=[config_file]     # 替代 rosparam command="load"
    )

    # 定义 RViz2 节点
    rviz_node = Node(
        package='rviz2',             # ROS 2 的 rviz 包名叫 rviz2
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] # 传递 RViz 配置文件参数
    )

    # 返回 LaunchDescription 对象
    return LaunchDescription([
        lvio_2d_node,
        rviz_node
    ])