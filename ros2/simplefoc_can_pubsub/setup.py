from setuptools import setup

package_name = 'simplefoc_can_pubsub'

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
    maintainer='jason',
    maintainer_email='jkirsons@gmail.com',
    description='SimpleFOC CAN motor topics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                    'talker = simplefoc_can_pubsub.publisher_member_function:main',
                    'listener = simplefoc_can_pubsub.subscriber_member_function:main',
                    'service = simplefoc_can_pubsub.service_member_function:main',
                    'client = simplefoc_can_pubsub.client_member_function:main',
            ],
    },
)
