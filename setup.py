from setuptools import setup, find_packages

setup(
    name='gym_gazebo3',
    version='0.1.0',
    description='Reinforcement Learning toolkit for ROS2 Humble and Gazebo Classic 11',
    author='Liam Ernst',
    author_email='wernst24@gmail.com',
    packages=find_packages(),
    install_requires=[
        'gymnasium>=0.26.0',
        'numpy>=1.21.0',
        'opencv-python>=4.5.0',
        'stable-baselines3>=1.6.0',
        'tensorboard>=2.8.0',
        'pyyaml>=5.4.0',
        'psutil>=5.8.0',
    ],
    python_requires='>=3.8',
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ],
)