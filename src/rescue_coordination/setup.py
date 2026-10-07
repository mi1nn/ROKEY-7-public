from setuptools import setup

package_name = 'rescue_coordination'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=[],
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='두 로봇의 경로 후보·임무 배정·접근 확인·통로 점유 Python 코어. ROS 어댑터는 미구현.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": []},
)
