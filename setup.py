from setuptools import setup

package_name = 'hri_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nila',
    maintainer_email='nila@example.com',
    description='UR5 HRI manipulation demo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
    'ik_node = hri_manipulation.ik_node:main',
'jacobian_torque_node = hri_manipulation.jacobian_torque_node:main',
'admittance_node = hri_manipulation.admittance_node:main',
        'lifting_service_node = hri_manipulation.lifting_service_node:main',
        'ur5_joint_motion_node = hri_manipulation.ur5_joint_motion_node:main', ],
# NEW for Task 4:
        'fsr_sensor = hri_manipulation.fsr_sensor:main',
        'force_mapping_node = hri_manipulation.mapping_node:main',
        'teleoperation_node = hri_manipulation.teleoperation:main',    },
)
