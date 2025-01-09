#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 런치 인자 선언
    agents_arg = DeclareLaunchArgument(
        'agents',
        default_value='20',
        description='시뮬레이션에 사용할 에이전트 수'
    )
    steps_arg = DeclareLaunchArgument(
        'steps',
        default_value='200',
        description='시뮬레이션의 스텝 수'
    )
    runs_arg = DeclareLaunchArgument(
        'runs',
        default_value='1000',
        description='시뮬레이션을 실행할 총 횟수'
    )

    # 런치 구성 변수 가져오기
    agents = LaunchConfiguration('agents')
    steps = LaunchConfiguration('steps')
    runs = LaunchConfiguration('runs')

    # 실행할 커맨드 정의
    

    # ExecuteProcess 액션 생성
    process = ExecuteProcess(
        cmd = [
            '/isaac-sim/python.sh',  # Isaac Sim용 Python 인터프리터 경로
            '/root/flow_ws/src/moving_people/src/py_social_force/generate_csv_psf.py',  # 수정된 스크립트 경로
            '--agents', agents,
            '--steps', steps,
            '--runs', runs,
        ],
        output='screen'
    )

    return LaunchDescription([
        agents_arg,
        steps_arg,
        runs_arg,
        process
    ])
