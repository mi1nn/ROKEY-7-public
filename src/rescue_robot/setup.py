from setuptools import setup

package_name = 'rescue_robot'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=['rescue_contracts==0.0.0', 'rescue_mapping==0.0.0'],
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='로봇별 명령·취소·정지 판정 Python 코어와 Nav2 명령 어댑터(ROS 경계는 ros_node).',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": ["command_adapter = rescue_robot.ros_node:main",
                                       "task_input = rescue_robot.task_input_node:main",
                                       "robot_task_input = rescue_robot.task_input_node:main",
                                       "command_watchdog = rescue_robot.watchdog_node:main",
                                       "task_prepare = rescue_robot.task_prepare:main",
                                       "clock_capture = rescue_robot.clock_capture:main"]},
)
