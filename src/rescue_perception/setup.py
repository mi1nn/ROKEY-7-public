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
    install_requires=["setuptools"],
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='카메라 입력·사람 탐지·거리 추정·관측 생성. 탐지와 ROS 입출력은 아직 없다.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": []},
)
