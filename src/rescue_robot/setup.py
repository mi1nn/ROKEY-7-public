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
    install_requires=["setuptools"],
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='로봇별 명령·취소·정지 판정 Python 코어. ROS 어댑터는 미구현.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": []},
)
