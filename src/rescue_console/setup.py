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
    # Flask is optional: aggregation and HTML rendering need only the standard library.
    extras_require={"web": ["flask"]},
    zip_safe=True,
    python_requires=">=3.10",
    maintainer="Maintainer not yet assigned",
    maintainer_email="maintainer@example.invalid",
    description='Read-only operator console view over the operations ledger; no ROS, no commands.',
    license="LicenseRef-Unspecified",
    entry_points={"console_scripts": ["rescue-console = rescue_console.__main__:main"]},
)
