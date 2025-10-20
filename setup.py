"""
Setup script for fmm (Fast Map Matching) Python package.

This package requires building C++ extensions via CMake.
"""

from pathlib import Path

from setuptools import find_packages, setup

# Read version
version_file = Path(__file__).parent / "python" / "fmm" / "_version.py"
if version_file.exists():
    exec(version_file.read_text())
    __version__ = locals().get("version", "0.1.0")
else:
    __version__ = "0.1.0"

# Read README
readme_file = Path(__file__).parent / "README.md"
long_description = readme_file.read_text() if readme_file.exists() else ""

setup(
    name="fmm",
    version=__version__,
    description="Fast Map Matching - High-performance map matching library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Can Yang",
    url="https://github.com/cyang-kth/fmm",
    license="MIT",
    packages=find_packages(where="python"),
    package_dir={"": "python"},
    # C++ extension is built by CMake and installed separately
    # This setup.py is for the Python wrapper code
    python_requires=">=3.7",
    # Optional dependencies for the MapMatcher helper
    extras_require={
        "matcher": [],
        "dev": ["pytest", "black", "mypy"],
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: C++",
        "Topic :: Scientific/Engineering :: GIS",
    ],
)
