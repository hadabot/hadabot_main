from setuptools import setup

package_name = 'hadabot_lesson'

setup(
    name=package_name, version='0.0.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']), ],
    install_requires=['setuptools'],
    zip_safe=True, maintainer='hadabot', maintainer_email='hadabot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration', tests_require=['pytest'],
    entry_points={
        'console_scripts':
        [
            'lab02_diff_drive = hadabot_lesson.lab02_diff_drive:main',
            'solution_lab02_diff_drive = hadabot_lesson.solution_lab02_diff_drive:main',
            'test_lab02_turtle_driver = hadabot_lesson.test_lab02_turtle_driver:main'], },)
