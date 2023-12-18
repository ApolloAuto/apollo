import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="aperf",
    version="0.0.2",
    author="apollo-team",
    author_email="apollo-support@baidu.com",
    description="Apollo's performance statistics tool",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ApolloAuto/apollo/tree/master/modules/tools/aperf",
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
    install_requires=[
        "matplotlib",
    ],
    entry_points={
        'console_scripts': [
            'aperf = aperf.main:main',
        ],
    },
    python_requires=">=3.6",
)
