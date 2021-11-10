from setuptools import setup

package_name = 'hadabot_lesson'

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
            'drive_straight_for_n_secs = hadabot_lesson.drive_straight_for_n_secs:main',
            'drive_straight_for_n_cm = hadabot_lesson.drive_straight_for_n_cm:main',
            'turn_angle_deg = hadabot_lesson.turn_angle_deg:main',
            'drive_square_pattern = hadabot_lesson.drive_square_pattern:main',
            'solution_drive_square_pattern = hadabot_lesson.solution_drive_square_pattern:main'
        ],
    },
)
