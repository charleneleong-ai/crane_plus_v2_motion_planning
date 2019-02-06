#!/usr/bin/env python
###
# File Created: Wednesday, February 6th 2019, 9:05:13 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By:
# Last Modified:
###
import os
from os.path import isfile, join, split
import glob
import pandas as pd

def files_to_df(dir):
    """Returns pd df of fullpaths, paths and filenames
    
    Arguments:
        dir {str} -- Desired root dir, can be wildcard
    
    Returns:
        pd.df     -- DF of fullpaths, paths and filenames
    """
    files = pd.DataFrame([file for file in glob.glob(dir)], columns=["fullpath"])
    #    fullpath
    # 0  dir\*\*.csv
    # 1  dir\*\*.csv
    # ..
    files_split = files['fullpath'].str.rsplit(os.sep, 1, expand=True).rename(columns={0: 'path', 1: 'filename'})
    #    path       filename
    # 0  dir\..  *.csv
    # 1  dir\..  *.csv
    # ..
    files = files.join(files_split)
    #    fullpath     path     filename
    # 0  dir\..\*.csv   dir\..  *.csv
    # 1  dir\..\*.csv   dir\..  *.csv
    # ..
    num_run_dirs = len(files_split['path'].unique())
    print('# Files: %d # Runs: %d \n' % (len(files), num_run_dirs))

    return files

def list_files(startpath):
    """Lists file dir structure

    Arguments:
        startpath {str} -- Startpath
    """

    for root, dirs, files in os.walk(startpath):
        level = root.replace(startpath, '').count(os.sep)
        indent = ' ' * 4 * (level)
        print('{}{}/'.format(indent, os.path.basename(root)))
        subindent = ' ' * 4 * (level + 1)
        for f in files:
            print('{}{}'.format(subindent, f))

def list_num_files(startpath):
    """Lists number of files in runs
    
    Arguments:
        startpath {str} -- Startpath
    """

    num_dirs = []
    num_files = []

    for root, dirs, files in os.walk(startpath):
        for d in dirs:
            num_dirs.append(d)
        num_files.append(len(files))
    num_files.pop(0)

    for idx, d in enumerate(num_dirs):
        print(d + ': '+ str(num_files[idx]) + ' files')
    print('\n')