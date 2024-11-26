#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 'csv_path' 런치 인수를 선언
    csv_path_arg = DeclareLaunchArgument(
        'csv_path',
        default_value='/root/flow_ws/src/moving_people/src/py_social_force/csv/crowd_coordinates1.csv',
        description='군중 좌표를 포함하는 CSV 파일의 경로'
    )

    # Python 스크립트를 실행하기 위한 명령 정의
    process = ExecuteProcess(
        cmd=[
            '/isaac-sim/python.sh',  # Isaac Sim용 Python 인터프리터 경로
            '/root/flow_ws/install/moving_people/lib/moving_people/special_csv_crowd_people.py',
            '--csv_path', LaunchConfiguration('csv_path')
        ],
        output='screen'
    )

    return LaunchDescription([
        csv_path_arg,
        process
    ])
