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
    install_requires=["setuptools"],
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='SLAM 연동·좌표 변환·지도 보정 후 관측 유효성. SLAM과 TF 코드는 아직 없다.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": []},
)
