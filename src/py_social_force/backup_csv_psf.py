from pathlib import Path
import numpy as np
import random
import pysocialforce as psf
import matplotlib.pyplot as plt
import pandas as pd  # CSV 파일 저장을 위해 사용

# 이미지 및 CSV 파일을 저장할 디렉토리 생성
Path("images").mkdir(exist_ok=True)
Path("csv").mkdir(exist_ok=True)

def random_spawn_x(left_side=True):
    """왼쪽 또는 오른쪽에서 스폰할 때의 x-좌표를 랜덤으로 생성"""
    if left_side:
        return random.uniform(-30, 0)  # 왼쪽 (-30에서 0 사이)
    else:
        return random.uniform(0, 30)   # 오른쪽 (0에서 30 사이)

def random_goal_x(spawn_x):
    """스폰 위치에 따라 목표 x-좌표를 설정"""
    if spawn_x < 0:
        return random.uniform(25, 30)   # 오른쪽 목표
    else:
        return random.uniform(-30, -25) # 왼쪽 목표

def random_spawn_y():
    """-4.5에서 4.5 사이의 y-좌표를 랜덤으로 생성"""
    return random.uniform(-4.5, 4.5)  # 복도 내에서 스폰

if __name__ == "__main__":
    # 기본 파일 이름 설정
    base_name = "crowd_coordinates"

    # 전체 에이전트 수
    total_agents = 200  # 에이전트 수를 200으로 설정

    initial_state = []
    groups = []
    group_colors = []
    agent_labels = []
    group_start_index = 0
    remaining_agents = total_agents

    # 모든 에이전트가 할당될 때까지 그룹 생성
    while remaining_agents > 0:
        # 그룹 크기를 1에서 5 사이로 결정
        group_size = min(random.randint(1, 5), remaining_agents)
        remaining_agents -= group_size

        # 스폰 방향을 랜덤으로 결정
        left_side = random.choice([True, False])

        center_x = random_spawn_x(left_side=left_side)
        center_y = random_spawn_y()

        goal_x = random_goal_x(center_x)
        goal_y = random_spawn_y()

        # 그룹 내 공통 속도 생성
        group_speed = random.uniform(0.8, 1.2)  # 속도를 0.8 m/s에서 1.2 m/s 사이로 설정
        angle = np.arctan2(goal_y - center_y, goal_x - center_x)  # 목표 방향의 각도

        # 그룹 내 에이전트 생성
        group = []
        for _ in range(group_size):
            # 에이전트 위치
            pos_x = random.uniform(center_x - 0.5, center_x + 0.5)
            pos_y = random.uniform(center_y - 0.5, center_y + 0.5)

            # 에이전트 속도 (약간의 변동 추가)
            speed_variation = random.uniform(-0.05, 0.05)
            agent_speed = group_speed + speed_variation

            # 속도를 x와 y 성분으로 분해
            vel_x = agent_speed * np.cos(angle)
            vel_y = agent_speed * np.sin(angle)

            # 에이전트 상태 추가
            agent_state = [
                pos_x,   # x 위치
                pos_y,   # y 위치
                vel_x,   # x 속도
                vel_y,   # y 속도
                goal_x,  # x 목표
                goal_y   # y 목표
            ]
            group.append(agent_state)

        initial_state.extend(group)
        groups.append(list(range(group_start_index, group_start_index + group_size)))

        # 에이전트 라벨 추가
        agent_labels.extend(range(group_start_index, group_start_index + group_size))

        # 스폰 방향에 따라 색상 할당
        if left_side:
            group_colors.extend(["red"] * group_size)
        else:
            group_colors.extend(["blue"] * group_size)

        group_start_index += group_size

    initial_state = np.array(initial_state)

    # 복도 벽을 나타내는 장애물 정의
    obs = [
        [-30, 30, 5, 5],    # 상단 경계
        [-30, 30, -5, -5],  # 하단 경계
    ]

    # 시뮬레이터 초기화
    s = psf.Simulator(
        initial_state,
        groups=groups,
        obstacles=obs,
        config_file=Path(__file__).resolve().parent.joinpath("psf_config.toml"),
    )

    # 설정 파라미터 확인을 위한 출력
    print("사용 가능한 설정 속성:")
    print(dir(s.config))
    print("\n설정 세부 사항:")
    print(s.config)

    # 에이전트 수 및 시뮬레이션 단계 수
    num_agents = len(initial_state)
    num_steps = 300

    # 시뮬레이션 데이터를 저장할 리스트 초기화
    data = []

    # 시뮬레이션 실행 및 데이터 수집
    for step in range(num_steps):
        s.step()

        # 현재 시뮬레이션 시간을 계산
        try:
            # [simulation] 섹션에서 dt 접근 시도
            time = step * s.config.simulation.dt
        except AttributeError:
            try:
                # 최상위 섹션에서 dt 접근 시도
                time = step * s.config.dt
            except AttributeError:
                # dt가 없을 경우 기본값 사용 및 경고 출력
                time = step * 0.4

        # 현재 에이전트 상태 가져오기
        positions = s.peds.state.copy()
        # positions 배열은 (num_agents, 6) 형태입니다:
        # 0: x 위치, 1: y 위치, 2: x 속도, 3: y 속도, 4: x 목표, 5: y 목표

        for i in range(num_agents):
            agent_id = agent_labels[i]
            x = positions[i, 0]
            y = positions[i, 1]
            vx = positions[i, 2]
            vy = positions[i, 3]
            goal_x = positions[i, 4]
            goal_y = positions[i, 5]

            data.append({
                'time': time,
                'agent_id': agent_id,
                'x': x,
                'y': y,
                'vx': vx,
                'vy': vy,
                'goal_x': goal_x,
                'goal_y': goal_y
            })

    # 데이터를 데이터프레임으로 변환
    df = pd.DataFrame(data)

    # CSV 파일을 저장할 디렉토리 생성
    csv_dir = Path("csv")
    csv_dir.mkdir(exist_ok=True)

    # 모든 데이터를 하나의 CSV 파일로 저장
    output_csv_path = csv_dir / f"{base_name}.csv"
    df.to_csv(output_csv_path, index=False)
    print(f"모든 시뮬레이션 데이터를 '{output_csv_path}'에 저장했습니다.")

    # 시뮬레이션 결과 시각화 및 저장
    # GIF 파일을 같은 기본 이름으로 저장
    # Path 객체를 문자열로 변환하여 전달
    output_gif_path = Path("images") / base_name  # '.gif' 제외

    # 디버깅 출력: output_gif_path의 타입과 값 확인
    print(f"output_gif_path type: {type(output_gif_path)}, value: {output_gif_path}")

    with psf.plot.SceneVisualizer(s, str(output_gif_path)) as sv:
        sv.agent_colors = group_colors  # 에이전트 색상 설정

        plt.xlim(-30, 30)  # x축 범위 설정
        plt.ylim(-10, 10)  # y축 범위 설정
        sv.animate()
    print(f"시뮬레이션 GIF 파일을 '{output_gif_path}.gif'에 저장했습니다.")
