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
    install_requires=["setuptools"],
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='두 로봇 수색·임무 배정·대상 인계·통행 조정. 배정 알고리즘과 ROS 입출력은 아직 없다.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": []},
)
