import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="amodel",
    version="0.2.1",
    author="apollo-team",
    author_email="apollo-support@baidu.com",
    description="Apollo's model deployment and management tool",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ApolloAuto/apollo/tree/master/modules/tools/amodel",
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
    install_requires=[],
    entry_points={
        'console_scripts': [
            'amodel = amodel.main:main',
        ],
    },
    python_requires=">=3.6",
)
