from setuptools import setup

package_name = 'rescue_mapping'

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
    description='순수 Python 3D 좌표·원관측·지도 revision 검증. ROS/SLAM/TF 실행 연동은 없다.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": []},
)
