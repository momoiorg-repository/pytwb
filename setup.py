from setuptools import setup, find_packages

install_requires = ["setuptools", "pydot"]

setup(
    name='pytwb',
    version='0.0.1',
    packages=['pytwb', 'pytwb.built_in_command', 'pytwb.built_in_bt'],
    author='Kazuya Tago',
    maintainer='Kazuya Tago <ktago.gm@gmail.com>',
    url='',
    keywords=['ROS', 'behaviour-trees'],
    zip_safe=True,
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries'
    ],
    description=(
        "A development tool to create ROS package based on "
        "behaviour trees framework."
    ),
    long_description=(
        "A development tool to create ROS package based on "
        "behaviour trees framework."
    ),
    license='BSD',
    package=find_packages(exclude=['pytwb_test*']),
    install_requires=[
        'py_trees',
        'py_trees_ros',
    ],
    entry_points={
        'console_scripts': [
            'pytwb = pytwb.lib_main:cli',
        ],
    },
)
