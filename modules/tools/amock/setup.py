import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="amock",
    version="0.1.0",
    author="wheelos",
    author_email="daohu527@gmail.com",
    description="Apollo mock messages",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/wheelos/apollo",
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
    package_data={},
    install_requires=[
        "keyboard",
        "cyber",
        "record_msg",
    ],
    entry_points={
        'console_scripts': [
            'amock = amock.main:main',
        ],
    },
    python_requires=">=3.6",
)
