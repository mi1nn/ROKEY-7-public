from setuptools import setup
package_name = 'rescue_contracts'
setup(name=package_name, version='0.0.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),
                  ('share/'+package_name,['package.xml','README.md'])],
      install_requires=[], python_requires='>=3.10', zip_safe=True,
      maintainer='Maintainer not yet assigned', maintainer_email='maintainer@example.invalid',
      description='ROS 없는 공통 오류·로봇 ID·정수 ns·임무 수락 계약.',
      license='LicenseRef-Unspecified')
