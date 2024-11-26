from pathlib import Path
import numpy as np
import random
import pysocialforce as psf
import matplotlib.pyplot as plt
import pandas as pd  # For saving CSV files
import argparse

# 절대 경로를 사용하여 'images'와 'csv' 디렉토리를 생성합니다.
base_path = Path("/root/flow_ws/src/moving_people/src/py_social_force")
images_path = base_path / "images"
csv_path = base_path / "csv"

# 부모 디렉토리가 없을 경우 함께 생성
images_path.mkdir(parents=True, exist_ok=True)
csv_path.mkdir(parents=True, exist_ok=True)


def random_spawn_x(left_side=True):
    """좌측 또는 우측에서 랜덤한 x 좌표를 생성합니다."""
    if left_side:
        return random.uniform(-30, 0)  # 좌측 (-30에서 0)
    else:
        return random.uniform(0, 30)   # 우측 (0에서 30)


def random_goal_x(spawn_x):
    """스폰 위치에 따라 목표 x 좌표를 설정합니다."""
    if spawn_x < 0:
        return random.uniform(25, 30)   # 우측 목표
    else:
        return random.uniform(-30, -25) # 좌측 목표


def random_spawn_y():
    """복도의 y 좌표 (-4.5에서 4.5) 내에서 랜덤한 y 좌표를 생성합니다."""
    return random.uniform(-4.5, 4.5)  # 복도 내


def run_simulation(run_number, total_agents=20, num_steps=300):
    """
    단일 시뮬레이션을 실행하고 결과를 저장합니다.

    Parameters:
    - run_number: int, 현재 시뮬레이션 실행 번호
    - total_agents: int, 시뮬레이션에 참여하는 에이전트 수
    - num_steps: int, 시뮬레이션 스텝 수
    """
    # 실행 번호를 포함한 기본 이름 정의
    base_name = f"crowd_coordinates{run_number}"

    initial_state = []
    groups = []
    group_colors = []
    agent_labels = []
    group_start_index = 0

    # 비율 선택: 30% ~ 70% 사이의 랜덤 비율
    left_ratio = random.uniform(0.3, 0.7)  # 좌측 비율 (30% ~ 70%)
    right_ratio = 1.0 - left_ratio         # 우측 비율

    print(f"[Run {run_number}] 선택된 비율 (좌:우) = {left_ratio:.2f}:{right_ratio:.2f}")

    # 각 측에 할당할 에이전트 수 계산 (반올림하여 정수로 변환)
    left_agents = round(left_ratio * total_agents)
    right_agents = total_agents - left_agents  # 남은 에이전트는 우측

    print(f"[Run {run_number}] 좌측 에이전트 수: {left_agents}, 우측 에이전트 수: {right_agents}")

    # 그룹별로 에이전트를 분배하기 위해 남은 에이전트 수를 추적
    remaining_left = left_agents
    remaining_right = right_agents

    # 에이전트의 초기 상태 생성
    while remaining_left > 0 or remaining_right > 0:
        # 그룹 크기 결정 (1~5, 남은 에이전트 수에 따라 조정)
        max_group_size = min(5, remaining_left + remaining_right)
        group_size = min(random.randint(1, 5), remaining_left + remaining_right)

        # 비율에 따라 좌측 또는 우측 선택
        if remaining_left > 0 and remaining_right > 0:
            # 현재 비율에 따라 확률적으로 측 선택
            prob_left = remaining_left / (remaining_left + remaining_right)
            left_side = random.random() < prob_left
        elif remaining_left > 0:
            left_side = True
        else:
            left_side = False

        # 그룹이 할당될 측의 남은 에이전트 수에 따라 그룹 크기 조정
        if left_side:
            group_size = min(group_size, remaining_left)
        else:
            group_size = min(group_size, remaining_right)

        # 그룹 크기가 0일 경우 루프 종료
        if group_size <= 0:
            break

        if left_side:
            remaining_left -= group_size
        else:
            remaining_right -= group_size

        center_x = random_spawn_x(left_side=left_side)
        center_y = random_spawn_y()

        goal_x = random_goal_x(center_x)
        goal_y = random_spawn_y()

        group_speed = random.uniform(0.8, 1.2)  # 속도는 0.8에서 1.2 m/s 사이
        angle = np.arctan2(goal_y - center_y, goal_x - center_x)  # 방향 각도

        group = []
        for _ in range(group_size):
            pos_x = random.uniform(center_x - 0.5, center_x + 0.5)
            pos_y = random.uniform(center_y - 0.5, center_y + 0.5)

            speed_variation = random.uniform(-0.05, 0.05)
            agent_speed = group_speed + speed_variation

            vel_x = agent_speed * np.cos(angle)
            vel_y = agent_speed * np.sin(angle)

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
        agent_labels.extend(range(group_start_index, group_start_index + group_size))

        # 스폰 측면에 따라 색상 할당
        if left_side:
            group_colors.extend(["red"] * group_size)
        else:
            group_colors.extend(["blue"] * group_size)

        group_start_index += group_size

    initial_state = np.array(initial_state)

    # 복도 벽을 장애물로 정의
    obs = [
        [-30, 30, 5, 5],    # 상단 경계
        [-30, 30, -5, -5],  # 하단 경계
    ]

    # 시뮬레이터 초기화
    config_path = base_path / "psf_config.toml"
    s = psf.Simulator(
        initial_state,
        groups=groups,
        obstacles=obs,
        config_file=str(config_path),
    )

    num_agents = len(initial_state)

    # 시뮬레이션 데이터를 저장할 리스트
    data = []

    # 시뮬레이션 실행
    for step in range(num_steps):
        s.step()

        # 현재 시뮬레이션 시간 계산
        try:
            time = step * s.config.simulation.dt
        except AttributeError:
            try:
                time = step * s.config.dt
            except AttributeError:
                time = step * 0.4  # dt를 찾을 수 없을 경우 기본값

        # 현재 에이전트 상태 가져오기
        positions = s.peds.state.copy()

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

    # 데이터를 DataFrame으로 변환
    df = pd.DataFrame(data)

    # CSV 저장 경로 설정 (절대 경로 사용)
    output_csv_path = csv_path / f"{base_name}.csv"
    try:
        df.to_csv(output_csv_path, index=False)
        print(f"[Run {run_number}] 시뮬레이션 데이터가 '{output_csv_path}'에 저장되었습니다.")
    except Exception as e:
        print(f"[Run {run_number}] CSV 저장 중 오류가 발생했습니다: {e}")

    # 시각화 및 GIF 생성
    output_gif_path = images_path / base_name  # '.gif'는 시각화 도구가 자동으로 추가

    try:
        with psf.plot.SceneVisualizer(s, str(output_gif_path)) as sv:
            sv.agent_colors = group_colors  # 에이전트 색상 설정

            plt.xlim(-30, 30)  # x축 한계 설정
            plt.ylim(-10, 10)  # y축 한계 설정
            sv.animate()
        print(f"[Run {run_number}] 시뮬레이션 GIF가 '{output_gif_path}.gif'에 저장되었습니다.")
    except Exception as e:
        print(f"[Run {run_number}] GIF 생성 중 오류가 발생했습니다: {e}")


def main():
    # 명령줄 인자 파서 생성
    parser = argparse.ArgumentParser(description="Pysocialforce Simulation Script")
    parser.add_argument(
        '--agents', '-a',
        type=int,
        default=20,
        help="시뮬레이션에 사용할 에이전트 수입니다. 기본값은 20입니다."
    )
    parser.add_argument(
        '--steps', '-s',
        type=int,
        default=300,
        help="시뮬레이션의 스텝 수입니다. 기본값은 300입니다."
    )
    parser.add_argument(
        '--runs', '-r',
        type=int,
        default=1,
        help="시뮬레이션을 실행할 총 횟수입니다. 기본값은 1입니다."
    )

    # 인자 파싱
    args = parser.parse_args()

    total_agents = args.agents
    num_steps = args.steps
    total_runs = args.runs

    # 입력값 검증
    if total_agents <= 0:
        print("에이전트 수는 1 이상이어야 합니다.")
        return
    if num_steps <= 0:
        print("시뮬레이션 스텝 수는 1 이상이어야 합니다.")
        return
    if total_runs <= 0:
        print("총 실행 횟수는 1 이상이어야 합니다.")
        return

    for run in range(1, total_runs + 1):
        print(f"시뮬레이션 실행 {run}/{total_runs}을(를) 시작합니다. 에이전트 수: {total_agents}, 스텝 수: {num_steps}")
        run_simulation(run_number=run, total_agents=total_agents, num_steps=num_steps)
        print(f"시뮬레이션 실행 {run}/{total_runs}이(가) 완료되었습니다.\n")
    
    print("모든 시뮬레이션이 성공적으로 완료되었습니다.")


if __name__ == "__main__":
    main()
