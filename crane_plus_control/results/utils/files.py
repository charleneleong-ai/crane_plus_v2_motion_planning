#!/usr/bin/env python
###
# File Created: Wednesday, February 6th 2019, 9:05:13 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By:
# Last Modified:
###
import os
import glob
import pandas as pd

def unique(dir):
    """Returns all unique filenames in given dir
    
    Arguments:
        dir {str} -- Desired root dir, can be wildcard
    
    Returns:
        list(str) -- List of unique filenames
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

    print('# Files: %d # Dirs: %d ' % (len(files), len(files_split['path'].unique())))

    return files['filename'].unique()