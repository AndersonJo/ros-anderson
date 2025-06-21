from setuptools import setup

package_name = 'turtlebot3_rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtlebot3_rl_env.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='TurtleBot3 Reinforcement Learning Environment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_environment = turtlebot3_rl.rl_environment:main',
            'simple_navigator = turtlebot3_rl.simple_navigator:main',
        ],
    },
)
