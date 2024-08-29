from setuptools import setup

package_name = 'dubal_eye_AND_web'

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
    maintainer_email='tlsgksfbd@seoultech.ac.kr',
    description='Description of dubal_eye_AND_web package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_subscriber = dubal_eye_AND_web.main:main',
        ],
    }
)

