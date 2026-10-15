from glob import glob

from setuptools import setup

package_name = 'rescue_bringup'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    # Nav2 trees without BackUp (OPS-207), read from the package folder by nav2_recovery.bt_dir()
    package_data={package_name: ["behavior_trees/*.xml"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", sorted(glob("launch/*.launch.py"))),
    ],
    install_requires=[],
    extras_require={"offline": ["rescue_contracts==0.0.0", "rescue_robot==0.0.0", "rescue_mapping==0.0.0",
                                "rescue_perception==0.0.0", "rescue_coordination==0.0.0",
                                "rescue_operations==0.0.0"]},
    zip_safe=False,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='정적 배포 검사, 합성 통합 리허설과 수동 실습용 TurtleBot4 SLAM/Nav2 실행 구성.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": ["check_deployment = rescue_bringup.cli:main",
                                    "slam_active_check = rescue_bringup.slam_active:main"]},
)
