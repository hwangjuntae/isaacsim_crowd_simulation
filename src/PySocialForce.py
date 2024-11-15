#!/usr/bin/env python3

import numpy as np
import pysocialforce as psf

# 초기 상태 정의 (위치, 속도, 목적지 등)
initial_state = np.array([
    [0, 0, 0, 0, 10, 10],  # 첫 번째 보행자 (x, y, vx, vy, goal_x, goal_y)
    [5, 5, 0, 0, 15, 15],  # 두 번째 보행자
])

# 그룹 설정 (보행자 ID로 구성된 그룹 리스트)
groups = [
    [0],  # 첫 번째 그룹: 첫 번째 보행자만 포함
    [1],  # 두 번째 그룹: 두 번째 보행자만 포함
]

# 장애물 정의 (각 행이 (startx, endx, starty, endy))
obstacles = np.array([
    [1, 5, 3, 3],  # 첫 번째 장애물: 수평선 (x=3~7, y=3)
    [3, 7, 1, 1],  # 두 번째 장애물: 수직선 (x=3, y=3~7)
])

# 시뮬레이터 초기화
sim = psf.Simulator(state=initial_state, groups=groups, obstacles=obstacles)

# 시뮬레이션 50번 스텝 실행
sim.step(n=50)

# 시뮬레이션 결과 애니메이션 생성
with psf.plot.SceneVisualizer(sim, "simulation_output") as sv:
    sv.animate()
