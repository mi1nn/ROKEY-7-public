from setuptools import setup

package_name = 'rescue_perception'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["rescue_contracts==0.0.0"],
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='기록 RGB-D 정합·광학 위치·관측 최신성·잠정 사건을 검증하는 순수 Python 코어. 모델 추론·ROS 입출력은 미구현.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": []},
)
