from setuptools import setup

package_name = 'modubot_odometry'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odom_only.launch.py']),
        # adicione aqui seu rviz se tiver: ('share/' + package_name + '/rviz', ['rviz/odom_view.rviz']),
    ],
    install_requires=['setuptools','pyserial','tf2_ros'],
    zip_safe=True,
    maintainer='levir',
    maintainer_email='you@example.com',
    description='Odometria via serial (consome linhas "T dL dR" da ESP32) e publica /odom + /tf',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_odometry = modubot_odometry.serial_odometry_node:main',
        ],
    },
)

