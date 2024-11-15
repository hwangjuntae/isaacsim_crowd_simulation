#!/usr/bin/env python3

import os

def generate_launch_description():
    # Isaac Sim Python 환경 설정 경로

    # 기본 사용자 홈 경로 변수 설정
    home_path = "/home/teus"

    # 경로를 변수로 설정
    isaac_python_path = f"{home_path}/.local/share/ov/pkg/isaac-sim-4.1.0/python.sh"
    script_path = f"{home_path}/flow_ws/src/moving_people/src/circle_people.py"

    # os.system을 사용하여 실행
    os.system(f"{isaac_python_path} {script_path}")
