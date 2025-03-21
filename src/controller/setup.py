from setuptools import setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daniel@undertheoak.net',
    description='Controller for my ROS2 RS Bot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hl_ctrl = controller.hl_ctrl:main',
            'mtr_ctrl = controller.mtr_ctrl:main',
            'hw_interface = controller.hw_interface:main',
            'depth_processing = controller.depth_processing:main'
        ],
    },
)
