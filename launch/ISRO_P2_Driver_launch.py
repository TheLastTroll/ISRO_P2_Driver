import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 패키지 경로 찾기
    pkg_share = FindPackageShare('ISRO_P2_Driver')

    # 1. 'mode' 인자 선언 (예: serial, client, server)
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='serial',
        description='Connection mode (serial, client, server)'
    )

    # 2. 설정 파일 경로 자동 완성
    # 입력받은 mode 값 뒤에 "_mode.yaml"을 붙여서 파일 경로를 만듭니다.
    # 예: mode:=client  -->  .../config/client_mode.yaml
    config_file_path = PathJoinSubstitution([
        pkg_share,
        'config',
        PythonExpression(["'", LaunchConfiguration('mode'), "_mode.yaml'"])
    ])

    # 3. 노드 실행
    pva_node = Node(
        package='ISRO_P2_Driver',
        executable='ISRO_P2_Driver_node',
        name='ISRO_P2_Driver_node',
        output='screen',
        parameters=[config_file_path],
        emulate_tty=True
    )

    return LaunchDescription([
        mode_arg,
        pva_node
    ])