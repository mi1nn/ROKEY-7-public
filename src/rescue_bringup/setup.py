from setuptools import setup

package_name = 'rescue_bringup'

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
    description='실행 프로파일 검사. 현재는 ROS를 import하지 않는 정적 JSON 검사 CLI만 동작한다.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": ["check_deployment = rescue_bringup.cli:main"]},
)
