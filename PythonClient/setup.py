from pathlib import Path
import os
import re

import setuptools

root = Path(__file__).resolve().parent
os.chdir(root)
version = re.search(r'^__version__ = "([^"]+)"', (root / 'airsim/__init__.py').read_text(encoding='utf-8'), re.MULTILINE).group(1)

setuptools.setup(
    name='airsim',
    version=version,
    author='Shital Shah',
    author_email='shitals@microsoft.com',
    description='AirsimX-compatible Python client for the AirSim simulation APIs',
    long_description=(root / 'README.md').read_text(encoding='utf-8'),
    long_description_content_type='text/markdown',
    url='https://github.com/naipi11/AirsimX',
    packages=setuptools.find_packages(exclude=('tests', 'tests.*')),
    license='MIT',
    python_requires='>=3.10',
    classifiers=(
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ),
    install_requires=[
        'msgpack>=1.0.7,<2',
        'numpy>=1.26,<3',
        'opencv-contrib-python>=4.8,<5',
    ],
    # The Gymnasium environment lives in the same distribution; its only extra
    # dependency is declared here so a plain client install stays lean.
    extras_require={
        'gym': ['gymnasium>=1.0,<2'],
    },
)
