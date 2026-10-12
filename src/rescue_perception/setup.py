from setuptools import setup
from glob import glob

package_name = 'rescue_perception'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    # 예시 파라미터는 패키지 안에도 둔다: 설치본 시험이 importlib.resources 로 읽는다.
    package_data={package_name: ["config/webcam_node.example.yaml", "config/*.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", [package_name + "/config/webcam_node.example.yaml"] + glob("config/*.yaml") + glob("config/*.json")),
    ],
    install_requires=["rescue_contracts==0.0.0", "rescue_mapping==0.0.0"],
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='기록 RGB-D 정합·광학 위치·관측 최신성·잠정 사건을 검증하는 순수 Python 코어. ROS 경계는 고정 웹캠·AMR 미리보기·RGB-D 좌표 시험 노드다.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": ["webcam_node = rescue_perception.webcam_node:main",
        "amr_camera_node = rescue_perception.amr_camera_node:main",
        "amr_target_node = rescue_perception.amr_target_node:main"]},
)
