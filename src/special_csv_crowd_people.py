#!/usr/bin/env python3
"""
| File: special_csv_crowd_people.py
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Pegasus API를 사용하여 사람들이 이동하는 시뮬레이션을 실행하는 앱을 빌드하는 예제 파일입니다.
"""

# Isaac Sim을 시작하기 위한 모듈 임포트
import carb
from isaacsim import SimulationApp

# Isaac Sim의 시뮬레이션 환경을 시작합니다.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# 실제 스크립트는 여기에서 시작됩니다
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.utils.extensions import disable_extension, enable_extension
import omni.usd

# 확장 활성화 코드
EXTENSIONS_TO_ENABLE = [
    'omni.anim.people',
    'omni.anim.navigation.bundle',
    'omni.anim.timeline',
    'omni.anim.graph.bundle',
    'omni.anim.graph.core',
    'omni.anim.graph.ui',
    'omni.anim.retarget.bundle',
    'omni.anim.retarget.core',
    'omni.anim.retarget.ui',
    'omni.kit.scripting',
    'omni.graph.io',
    'omni.anim.curve.core',
    'omni.isaac.ros2_bridge'
]

# 필요한 확장 활성화
for ext in EXTENSIONS_TO_ENABLE:
    enable_extension(ext)

# 사용하지 않는 확장 비활성화
disable_extension("omni.isaac.ros_bridge")

# 새로운 확장으로 시뮬레이션 앱 업데이트
simulation_app.update()

# 새로운 USD 스테이지를 재시작하여 사람 확장이 로드되었는지 확인합니다
omni.usd.get_context().new_stage()

# 이제 Pegasus API 및 관련 모듈을 임포트합니다
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.people.person import Person
from pegasus.simulator.logic.people.person_controller import PersonController

class CSVPersonController(PersonController):
    def __init__(self, agent_id, csv_data):
        super().__init__()
        self.agent_id = agent_id
        self.csv_data = csv_data
        self.current_index = 0
        self._world = World.instance()  # 월드 인스턴스 가져오기

    def update(self, dt: float):
        # 현재 시뮬레이션 시간 가져오기
        current_time = self._world.current_time  # 수정된 부분

        # 시간에 따라 위치 업데이트
        while self.current_index < len(self.csv_data) and self.csv_data.iloc[self.current_index]['time'] <= current_time:
            self.current_index += 1

        if self.current_index == 0:
            # 아직 시작되지 않음
            return
        elif self.current_index >= len(self.csv_data):
            # 데이터 끝에 도달
            return
        else:
            # 현재 시간에 해당하는 데이터 가져오기
            row = self.csv_data.iloc[self.current_index - 1]
            target_position = [row['x'], row['y'], 0.0]
            self._person.update_target_position(target_position)

class PegasusApp:
    """
    Isaac Sim 독립 실행형 앱의 템플릿 클래스.
    """

    def __init__(self):
        """
        PegasusApp을 초기화하고 시뮬레이션 환경을 설정하는 메서드입니다.
        """

        # 시뮬레이션을 시작/중지할 타임라인 설정
        self.timeline = omni.timeline.get_timeline_interface()

        # Pegasus 인터페이스 시작
        self.pg = PegasusInterface()

        # 물리학 설정 및 자산 생성용 World 객체 초기화
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # 지정된 USD 파일 로드
        usd_path = "/root/flow_ws/src/moving_people/usd/straight.usd"
        self.pg.load_asset(usd_path, "/World/layout")

        # CSV 파일 경로
        csv_path = "/root/flow_ws/src/moving_people/src/py_social_force/csv/crowd_coordinates1.csv"

        # CSV 데이터 읽기
        self.csv_data = pd.read_csv(csv_path)

        # 에이전트 ID 리스트 추출
        agent_ids = self.csv_data['agent_id'].unique()

        # 에이전트별 데이터 분리
        self.agents_data = {}
        for agent_id in agent_ids:
            self.agents_data[agent_id] = self.csv_data[self.csv_data['agent_id'] == agent_id].reset_index(drop=True)

        # 에이전트 딕셔너리
        self.agents = {}

        # 각 에이전트에 대해 Person 객체 생성
        for agent_id, data in self.agents_data.items():
            # 예시로 남성 및 여성 캐릭터를 번갈아가며 사용
            if int(agent_id) % 2 == 0:
                asset_name = "original_male_adult_construction_05"
            else:
                asset_name = "original_female_adult_business_02"

            # 초기 위치는 CSV의 첫 번째 데이터 포인트 사용
            init_x = data.iloc[0]['x']
            init_y = data.iloc[0]['y']

            # CSV 기반 컨트롤러 생성
            controller = CSVPersonController(agent_id, data)

            # Person 객체 생성 시 컨트롤러를 전달
            person = Person(
                f"person_{agent_id}",  # prim_path에서 선행 슬래시 제거
                asset_name,
                init_pos=[init_x, init_y, 0.0],
                init_yaw=0.0,
                controller=controller  # 컨트롤러를 생성자에 전달
            )

            # 에이전트 딕셔너리에 추가
            self.agents[agent_id] = person

        self.stop_sim = False

    def run(self):
        """
        물리적 스텝을 실행하는 메인 루프.
        """

        # 시뮬레이션 시작
        self.timeline.play()

        # "무한" 루프
        while simulation_app.is_running() and not self.stop_sim:
            # 앱의 UI를 업데이트하고 물리적 스텝을 수행합니다.
            self.world.step(render=True)

        # 정리 및 종료
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():
    # 템플릿 앱 인스턴스화
    pg_app = PegasusApp()

    # 애플리케이션 루프 실행
    pg_app.run()

if __name__ == "__main__":
    main()
