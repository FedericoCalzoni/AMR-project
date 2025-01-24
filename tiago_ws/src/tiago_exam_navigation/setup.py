from setuptools import find_packages, setup

package_name = 'tiago_exam_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='federicocalzoni01@gmail.com',
    description='Exam-specific navigation code for Tiago robot',
    license='Apache-2.0',  # Update to match your license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_to_pose = tiago_exam_navigation.navigate_to_pose:main',
            'task_manager = tiago_exam_navigation.task_manager:main',
        ],
    },
)