#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import random
import pysocialforce as psf
import matplotlib.pyplot as plt

# 이미지를 저장할 디렉토리 생성
Path("images").mkdir(exist_ok=True)

def random_spawn_x(left_side=True):
    """왼쪽 또는 오른쪽에서 스폰될 x 좌표를 랜덤하게 생성합니다."""
    if left_side:
        return random.uniform(-30, 0)  # 왼쪽 스폰 (-30부터 0까지)
    else:
        return random.uniform(0, 30)   # 오른쪽 스폰 (0부터 30까지)

def random_goal_x(spawn_x):
    """스폰 위치에 따라 목표 x 좌표를 설정합니다."""
    if spawn_x < 0:
        return random.uniform(25, 30)   # 목표는 오른쪽
    else:
        return random.uniform(-30, -25) # 목표는 왼쪽

def random_spawn_y():
    """-5부터 5 사이의 y 좌표를 생성합니다."""
    return random.uniform(-4.5, 4.5)  # 통로 내부에서 스폰

if __name__ == "__main__":
    # 총 인원 설정
    total_agents = 20  # 총 200명으로 설정

    initial_state = []
    groups = []
    group_colors = []
    group_start_index = 0
    remaining_agents = total_agents

    # 모든 에이전트가 할당될 때까지 그룹 생성
    while remaining_agents > 0:
        # 그룹 크기를 1에서 5 사이의 랜덤 값으로 결정
        group_size = min(random.randint(1, 5), remaining_agents)
        remaining_agents -= group_size

        # 스폰 방향을 랜덤하게 결정
        left_side = random.choice([True, False])

        center_x = random_spawn_x(left_side=left_side)
        center_y = random_spawn_y()

        goal_x = random_goal_x(center_x)
        goal_y = random_spawn_y()

        # 그룹 공통 속도 생성
        group_speed = random.uniform(0.8, 1.2)  # 그룹의 공통 속도 (0.8m/s ~ 1.2m/s)
        angle = np.arctan2(goal_y - center_y, goal_x - center_x)  # 목표 방향의 각도

        # 그룹 내 에이전트 생성
        group = []
        for _ in range(group_size):
            # 에이전트 위치
            pos_x = random.uniform(center_x - 0.5, center_x + 0.5)
            pos_y = random.uniform(center_y - 0.5, center_y + 0.5)

            # 에이전트 속도 (그룹 속도에 작은 변동 추가)
            speed_variation = random.uniform(-0.05, 0.05)
            agent_speed = group_speed + speed_variation

            # 속도를 x, y 성분으로 분해
            vel_x = agent_speed * np.cos(angle)
            vel_y = agent_speed * np.sin(angle)

            # 에이전트 상태 추가
            agent_state = [
                pos_x,  # x 위치
                pos_y,  # y 위치
                vel_x,  # x 속도
                vel_y,  # y 속도
                goal_x,  # x 목표
                goal_y   # y 목표
            ]
            group.append(agent_state)

        initial_state.extend(group)
        groups.append(list(range(group_start_index, group_start_index + group_size)))

        # 스폰 방향에 따라 색상 지정
        if left_side:
            group_colors.extend(["red"] * group_size)
        else:
            group_colors.extend(["blue"] * group_size)

        group_start_index += group_size

    initial_state = np.array(initial_state)

    # 통로 형태의 장애물 정의
    obs = [
        [-30, 30, 5, 5],    # 위쪽 경계
        [-30, 30, -5, -5],  # 아래쪽 경계
    ]

    # 시뮬레이터 초기화
    s = psf.Simulator(
        initial_state,
        groups=groups,
        obstacles=obs,
        config_file=Path(__file__).resolve().parent.joinpath("psf_config.toml"),
    )

    # 시뮬레이션 실행
    s.step(300)

    # 결과 시각화 및 저장
    with psf.plot.SceneVisualizer(s, "images/my_psf") as sv:
        sv.agent_colors = group_colors  # 에이전트 색상 설정

        plt.xlim(-30, 30)  # x축 범위 설정
        plt.ylim(-10, 10)  # y축 범위 설정
        sv.animate()
