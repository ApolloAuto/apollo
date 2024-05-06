import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="adataset",
    version="0.1.4",
    author="apollo-team",
    author_email="apollo-support@baidu.com",
    description="Dataset conversion to Apollo record tool",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ApolloAuto/apollo/tree/master/modules/tools/adataset",
    project_urls={
        "Bug Tracker": "https://github.com/ApolloAuto/apollo/issues",
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: OS Independent",
    ],
    package_dir={"": "."},
    packages=setuptools.find_packages(where="."),
    package_data={"": [
        'calibration_meta/camera_params/*.yaml',
        'calibration_meta/gnss_params/*.yaml',
        'calibration_meta/radar_params/*.yaml',
        'calibration_meta/vehicle_params/*.yaml',
        'calibration_meta/velodyne_params/*.yaml',
    ]},
    install_requires=[
        "numpy",
        "scipy",
        "pyproj",
        "cyber_record",
        "record_msg",
    ],
    entry_points={
        'console_scripts': [
            'adataset = adataset.main:main',
        ],
    },
    python_requires=">=3.6",
)
