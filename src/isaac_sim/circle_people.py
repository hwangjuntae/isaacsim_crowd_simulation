#!/usr/bin/env python3
"""
| File: 9_people.py
| License: BSD-3-Clause. Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
| Description: Pegasus API를 사용하여 사람들이 이동하는 시뮬레이션을 실행하는 앱을 빌드하는 예제 파일입니다.
"""

# Isaac Sim을 시작하기 위한 모듈 임포트
import carb
from isaacsim import SimulationApp

# Isaac Sim의 시뮬레이션 환경을 시작합니다.
# 주의: 시뮬레이션 앱은 이 위치에서 바로 인스턴스화해야 합니다.
# 그렇지 않으면 확장 및 시뮬레이터가 제대로 로드되지 않아 충돌이 발생합니다.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# 실제 스크립트는 여기에서 시작됩니다
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.core.utils.extensions import disable_extension, enable_extension

# 필요한 사람 관련 확장 목록
EXTENSIONS_PEOPLE = [
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
]

# 확장 활성화
for ext_people in EXTENSIONS_PEOPLE:
    enable_extension(ext_people)

# ROS2 Bridge 확장을 활성화하고 ROS Bridge 확장을 비활성화합니다
disable_extension("omni.isaac.ros_bridge")
enable_extension("omni.isaac.ros2_bridge")

# 새로운 확장으로 시뮬레이션 앱 업데이트
simulation_app.update()

# -------------------------------------------------------------------------------------------------
# 새 USD 스테이지를 재시작하여 사람 확장이 로드되었는지 확인합니다
# -------------------------------------------------------------------------------------------------
import omni.usd
omni.usd.get_context().new_stage()

import numpy as np

# Pegasus API를 통해 드론 시뮬레이션을 위한 모듈 임포트
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.people.person import Person
from pegasus.simulator.logic.people.person_controller import PersonController
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# 원을 그리며 이동하는 사람 제어를 위한 예제 컨트롤러 클래스
# 참고: 이곳에 다른 행동을 위한 제어기를 추가할 수 있습니다.
class CirclePersonController(PersonController):

    def __init__(self):
        super().__init__()
        self._radius = 5.0  # 이동 반경 설정
        self.gamma = 0.0    # 각도 초기화
        self.gamma_dot = 0.3  # 각속도 설정
        
    def update(self, dt: float):
        # 사람의 참조 위치 갱신
        self.gamma += self.gamma_dot * dt
        
        # 사람이 따라갈 목표 위치 설정
        self._person.update_target_position([self._radius * np.cos(self.gamma), self._radius * np.sin(self.gamma), 0.0])

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# -------------------------------------------------------------------------------------------------
# PegasusApp 클래스 정의 및 시뮬레이션 실행
# -------------------------------------------------------------------------------------------------
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

        # NVIDIA 제공 월드 중 하나를 로드
        # self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        self.pg.load_asset(SIMULATION_ENVIRONMENTS["Curved Gridroom"], "/World/layout")

        # 사용 가능한 사람 자산 목록 확인
        people_assets_list = Person.get_character_asset_list()
        for person in people_assets_list:
            print(person)

        # 원형 궤도로 이동하는 사람 컨트롤러 생성
        person_controller = CirclePersonController()
        p1 = Person("person1", "original_male_adult_construction_05", init_pos=[3.0, 0.0, 0.0], init_yaw=1.0, controller=person_controller)
        
        # 수동 목표 위치를 설정하는 사람 생성
        p2 = Person("person2", "original_female_adult_business_02", init_pos=[2.0, 0.0, 0.0])
        p2.update_target_position([10.0, 0.0, 0.0], 1.0)

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
