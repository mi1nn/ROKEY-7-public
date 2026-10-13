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
    install_requires=["rescue_contracts==0.0.0"],
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='Report ledger, evidence, outbox and site-state DB; DB writer and report receipt nodes are the ROS boundary.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": ["rescue-report-export = rescue_operations.__main__:main",
                                    "db_writer = rescue_operations.ros_node:main",
                                    "report_node = rescue_operations.report_node:main",
                                    "map_snapshot = rescue_operations.map_snapshot:main"]},
)
