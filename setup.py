
from setuptools import setup, find_packages

setup(
    name='ros2_asyncio',
    version='0.1.0',
    maintainer='Aescape Inc',
    author='Tobias Lang',
    author_email='tobias@aescape.com',
    description='ros2_asyncio is a library to write Ros2 compatible concurrent code using the async/await syntax',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/aescape-inc/ros2_asyncio',
    packages=find_packages(),
    install_requires=[
        'rclpy',
        'pytest'
    ],
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Topic :: Software Development :: Libraries',
        'Topic :: System :: Networking'
    ],
    python_requires='>=3.7',
)
