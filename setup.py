# Copyright (C) 2019 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the 3-Clause BSD License which accompanies this
# distribution, and is available at
# https://opensource.org/licenses/BSD-3-Clause
#
# Contributors:
# Christoph Suerig <christoph.suerig@dlr.de>
import os
import sys
from distutils.core import setup
from setuptools import find_packages

PACKAGE_NAME = 'rafcon-task-planner-plugin'


def get_readme():
    return open(os.path.join(os.path.dirname(__file__), 'README.md'), 'r').read()

def get_data_files():
  data_files_dir = 'examples'
  data_files = []
  for (cur_dir, contained_dirs, contained_files) in os.walk(data_files_dir):
    files_in_cur_dir = []
    for file in contained_files:
      files_in_cur_dir.append(os.path.join(cur_dir,file))
    data_files.append((os.path.join('share','rafcontpp',cur_dir),files_in_cur_dir))
  return data_files



setup(
  name = PACKAGE_NAME,
  version = '1.5.2',
  license='BSD-3-Clause',
  description = 'The RAFCON Task Planner Plugin (RTPP) is a plugin to interface arbitrary pddl planner'
                ' and automate the state machine generation process.',
  long_description=get_readme(),
  long_description_content_type='text/markdown',
  author = 'Christoph Suerig, Sebastian Brunner',
  author_email = 'christoph.suerig@dlr.de, sebastian.brunner@dlr.de',
  url = 'https://dlr-rm.github.io/rafcon-task-planner-plugin/',
  packages=find_packages("source"),
  include_package_data=True,
  package_dir={'': "source"}, # tell distutils packages are under source
  package_data={
        # Include all glade files
        'rafcontpp.view.glade': ['*.glade']
  },
  data_files = get_data_files(),
  #install_requires=['rafcon'],
  keywords = ['RAFCON', 'PDDL', 'Planner', 'state machine', 'robotic', 'FSM', 'development', 'GUI'],
  classifiers=[
    'Development Status :: 4 - Beta',
    'License :: OSI Approved :: BSD License',
    'Natural Language :: English',
    'Operating System :: Unix',
    'Programming Language :: Python :: 2.7',
    'Programming Language :: Python :: 3',
    'Environment :: X11 Applications :: GTK',
    'Intended Audience :: Developers',
    'Intended Audience :: Education',
    'Intended Audience :: Manufacturing',
    'Intended Audience :: Science/Research',
    'Topic :: Scientific/Engineering',
    'Topic :: Software Development',
    'Topic :: Utilities'
  ],
)
