from setuptools import setup

package_name = 'rescue_operations'

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
    description='관제·보고·기록의 경계. 웹·DB·ROS 입출력은 아직 없다.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": []},
)
