from setuptools import setup, find_packages

setup(
    name='sdk-py-location-transform',
    version='3.0.0',
    description=(
        'This application requests AI inference from HD Vision and '
        'transforms the pixel space to robot space.'
    ),
    author='Riley Duffy',
    license='MIT',
    packages=['hdv','location_transform','hdv.object_detection2d','location_transform.location_data','appdata','app_server','api_helper','vision'],
    install_requires=[
        'PyJWT',
        'ctrlx-datalayer<=3.5',
        'ctrlx-fbs',
        'numpy',
        'flatbuffers',
        'opencv-python-headless'
    ],
    scripts = ['main.py']
)