from setuptools import setup

package_name = 'hadabot_lesson_py'

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
    maintainer='hadabot',
    maintainer_email='hadabot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ir_sensor_driver = hadabot_lesson_py.ir_sensor_driver:main',
            ('solution_ir_sensor_driver = '
             'hadabot_lesson_py.solution_ir_sensor_driver:main')
        ],
    },
)
