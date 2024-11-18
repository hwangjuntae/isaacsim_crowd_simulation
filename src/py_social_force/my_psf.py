#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import random
import pysocialforce as psf
import matplotlib.pyplot as plt

# 이미지 저장 디렉토리 생성
Path("images").mkdir(exist_ok=True)

def random_spawn_x(left_side=True):
    """왼쪽 또는 오른쪽 스폰 x 좌표를 랜덤으로 생성"""
    if left_side:
        return random.uniform(-25, -20)  # 왼쪽 스폰 (-25 ~ -20)
    else:
        return random.uniform(20, 25)  # 오른쪽 스폰 (20 ~ 25)

def random_goal_x(spawn_x):
    """스폰 위치에 따라 목표 x 좌표를 설정"""
    if spawn_x < 0:
        return random.uniform(20, 25)  # 목표는 오른쪽
    else:
        return random.uniform(-25, -20)  # 목표는 왼쪽

def random_spawn_y():
    """y 좌표를 -5~5 범위에서 생성"""
    return random.uniform(-4.5, 4.5)  # 통로 내부에서 스폰

if __name__ == "__main__":
    # 총 인원 설정 변수
    total_agents = 100  # 설정 가능한 총 인원
    max_left_agents = total_agents // 2  # 왼쪽에서 생성될 인원
    max_right_agents = total_agents // 2  # 오른쪽에서 생성될 인원

    # 그룹 수 설정 (왼쪽과 오른쪽 각각)
    num_left_groups = random.randint(3, 7)  # 왼쪽 그룹 수
    num_right_groups = random.randint(3, 7)  # 오른쪽 그룹 수

    initial_state = []
    groups = []
    group_colors = []  # 보행자 색상 저장
    group_start_index = 0

    # 왼쪽 그룹 생성
    remaining_left_agents = max_left_agents
    for group_id in range(num_left_groups):
        if remaining_left_agents <= 0:
            break

        group_size = min(random.randint(5, 15), remaining_left_agents)
        remaining_left_agents -= group_size

        center_x = random_spawn_x(left_side=True)
        center_y = random_spawn_y()

        goal_x = random_goal_x(center_x)
        goal_y = random_spawn_y()

        group = [
            [random.uniform(center_x - 0.5, center_x + 0.5),
             random.uniform(center_y - 0.5, center_y + 0.5),
             random.uniform(-0.5, 0.5),
             random.uniform(-0.5, 0.5),
             goal_x,
             goal_y]
            for _ in range(group_size)
        ]

        initial_state.extend(group)
        groups.append(list(range(group_start_index, group_start_index + group_size)))
        group_colors.extend(["red"] * group_size)  # 왼쪽 그룹은 빨간색
        group_start_index += group_size

    # 오른쪽 그룹 생성
    remaining_right_agents = max_right_agents
    for group_id in range(num_right_groups):
        if remaining_right_agents <= 0:
            break

        group_size = min(random.randint(5, 15), remaining_right_agents)
        remaining_right_agents -= group_size

        center_x = random_spawn_x(left_side=False)
        center_y = random_spawn_y()

        goal_x = random_goal_x(center_x)
        goal_y = random_spawn_y()

        group = [
            [random.uniform(center_x - 0.5, center_x + 0.5),
             random.uniform(center_y - 0.5, center_y + 0.5),
             random.uniform(-0.5, 0.5),
             random.uniform(-0.5, 0.5),
             goal_x,
             goal_y]
            for _ in range(group_size)
        ]

        initial_state.extend(group)
        groups.append(list(range(group_start_index, group_start_index + group_size)))
        group_colors.extend(["blue"] * group_size)  # 오른쪽 그룹은 파란색
        group_start_index += group_size

    initial_state = np.array(initial_state)

    # 통로 형태의 장애물 정의
    obs = [
        [-30, 30, 5, 5],  # 위쪽 경계
        [-30, 30, -5, -5],  # 아래쪽 경계
    ]

    # 시뮬레이터 초기화
    s = psf.Simulator(
        initial_state,
        groups=groups,
        obstacles=obs,
        # config_file=Path(__file__).resolve().parent.joinpath("example.toml"),
    )

    # 시뮬레이션 실행
    s.step(300)

    # 결과 시각화 및 저장
    with psf.plot.SceneVisualizer(s, "images/my_psf") as sv:

        # 보행자 색상 설정
        sv.agent_colors = group_colors  # 각 보행자의 색상 설정

        plt.xlim(-30, 30)  # x축 범위 설정
        plt.ylim(-10, 10)  # y축 범위 설정
        sv.animate()
