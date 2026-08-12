from setuptools import setup

package_name = 'espeak_speaker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/espeak_node']),
    ],
    scripts=['scripts/espeak_node'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lishuai',
    maintainer_email='lishuai@todo.todo',
    description='Subscribe to the /speak topic and announce the text with eSpeak on the local speaker.',
    license='Apache-2.0',
)
