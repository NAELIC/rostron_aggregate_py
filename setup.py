from setuptools import setup

package_name = 'rostron_filters_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '/utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Etienne Schmitz',
    maintainer_email='contact@etienne-schmitz.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_filter = rostron_filters_py.simple_filter:main'
        ],
    },
)
