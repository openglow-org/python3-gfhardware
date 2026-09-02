from setuptools import setup, Extension

# gfhardware/src/bayer.c (Bayer demosaicing, from libdc1394) is
# LGPL-2.1-or-later and is compiled into gfhardware._cam alongside the MIT
# sources; see README.md and LICENSE.LGPL-2.1.
setup(
    name='gfhardware',
    description='Glowforge Hardware Support',
    author='Scott Wiederhold',
    author_email='s.e.wiederhold@gmail.com',
    url='https://github.com/openglow-org/python3-gfhardware',
    version='0.1.0',
    license='MIT AND LGPL-2.1-or-later',
    long_description=open('README.md').read(),
    keywords='Glowforge OpenGlow OV5648 imx6',
    packages=['gfhardware', 'gfhardware.input', 'gfhardware.utils'],
    install_requires=['gfutilities', 'requests'],
    ext_modules=[
        Extension(
            name='gfhardware._cam',
            sources=['gfhardware/src/gfcam.c', 'gfhardware/src/bayer.c'],
            libraries=["v4l2", "jpeg"]),
        Extension(
            name='gfhardware.input.evdev',
            sources=['gfhardware/src/evdev.c']),
    ],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'License :: OSI Approved :: GNU Lesser General Public License v2 or later (LGPLv2+)',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Operating System :: POSIX :: Linux',
        'Topic :: Software Development :: Embedded Systems',
    ],
)
