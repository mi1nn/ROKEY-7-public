from setuptools import setup

package_name = 'rescue_console'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["rescue_contracts==0.0.0", "rescue_operations==0.0.0"],
    # Flask는 선택 의존성이다: 데이터 집계 및 HTML 렌더링은 표준 라이브러리만으로 동작한다.
    extras_require={"web": ["flask"]},
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='Operator console: read-only view of the operations site state; sends operator commands to operations once; no ROS.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": ["rescue-console = rescue_console.__main__:main"]},
)
