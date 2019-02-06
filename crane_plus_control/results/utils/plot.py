#!/usr/bin/env python
###
# File Created: Wednesday, February 6th 2019, 9:06:21 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By:
# Last Modified:
###

#!/usr/bin/env python
###
# File Created: Wednesday, February 6th 2019, 8:48:56 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By:
# Last Modified:
###

# File handling libs
import os
from os.path import isfile, join, split
import glob
import csv

import pandas as pd

# Factor plot
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as mplcm
import matplotlib.colors as colors

# Timeseries data
import datetime
from pandas.tseries.frequencies import to_offset

from files import unique

# Linear Resampling irregular timeseries data
def resample(df, rate='T', short_rate='S', max_gap=None):
    """ Resample (unevenly spaced) timeseries data linearly by first upsampling to a
        high frequency (short_rate) then downsampling to the desired rate.
    
    :param df: a pandas timeseries object
    :param rate: rate that df should be resampled to
    :param short_rate: intermediate upsampling rate; if None, smallest interval of df is used
    :param max_gap: null intervals larger than `max_gap` are being treated as missing
        data and not interpolated. if None, always interpolate. must be provided as pandas
        frequency string format, e.g. '6h'
    """
    plt.rcParams['figure.figsize'] = [15, 10]
    
    # Resetting index to elapsed time 7200s
    # Setting first value to 0
    df.loc[-1] = df.loc[0]
    df.index = df.index + 1 
    df = df.sort_index() 
    df.loc[0,'elapsed_time'] = 0
    df.loc[0,'n_trial'] = 0

    # Setting last value to 7200
    df = df.loc[df['elapsed_time'] <= 7200]
    df = df.append(df.iloc[-1]).reset_index(drop=True)
    df.iloc[-1, df.columns.get_loc('elapsed_time')] = 7200
    df.iloc[-1, df.columns.get_loc('n_trial')] = df.iloc[-2, df.columns.get_loc('n_trial')]+1

    # Converting to timedelta and setting time as index
    df['elapsed_time'] = df['elapsed_time'].apply(lambda x : datetime.timedelta(seconds=x))
    df = df.set_index('elapsed_time')
    
    # return series if empty
    if df.empty:
        return df

    # check for timedelta index
    assert isinstance(
        df.index[0], pd.Timedelta), 'Object must have a datetime-like index.'

    # sort df by time
    df.sort_index(inplace=True)

    # create timedelta from frequency string
    rate_delta = to_offset(rate).delta

    # compute time intervals
    diff = np.diff(df.index) / np.timedelta64(1, 's')

    if max_gap is not None:
        # identify intervals in df larger than max_gap
        idx = np.where(np.greater(diff, to_offset(max_gap).delta.total_seconds()))[0]
        start = df.index[idx].tolist()
        stop = df.index[idx + 1].tolist()
        # store start and stop indices of large intervals
        big_gaps = list(zip(start, stop))

    if short_rate is None:
        # use minimal nonzero interval of original series as short_rate
        short_rate = '%dS' % diff[np.nonzero(diff)].min()
        # create timedelta from frequency string
        short_rate_delta = to_offset(short_rate).delta
        # if smallest interval is still larger than rate, use rate instead
        if short_rate_delta > rate_delta:
            short_rate = rate
    else:
        # convert frequency string to timedelta
        short_rate_delta = to_offset(short_rate).delta
        # make sure entered short_rate is smaller than rate
        assert rate_delta >= short_rate_delta, 'short_rate must be <= rate'

    # upsample to short_rate
    df = df.resample(short_rate).mean().interpolate()

    # downsample to desired rate
    df = df.resample(rate).ffill()

    # replace values in large gap itervals with NaN
    if max_gap is not None:
        for start, stop in big_gaps:
            df[start:stop] = None
    
    return df

def get_avg_losses():
    filenames = unique('final/*/*.csv')

    # Iterate over unique filenames; read CSVs, concat DFs
    avg_losses = pd.DataFrame()
    losses =pd.DataFrame()
    for fn in filenames:
       # fn = files['filename'].unique()[1]
        name = fn.split('.')[0]
        print(name)

        dir, f = split(fn)
        f = f.split('_')
        if any(x in fn for x in ['default', 'ompl']):
            mode = f[-1].split('.')[-2]
        else:
            planner_fn = f[-1].split('.')[-2]
            mode = f[2]  

        paths = files[files['filename'] == fn]['fullpath'] # Get list of fullpaths from unique filenames
        dfs = []
        
        losses=pd.DataFrame()
        for idx, path in enumerate(paths):
            if not any(x in path for x in ['default', 'ompl']):
                df = pd.read_csv(path, index_col=False, sep=',').dropna(axis=1)
                df_resample = resample(df, '2S')
                df_resample = df_resample['loss']     
                dfs.append(df_resample) 
            else:
                df = pd.read_csv(path, index_col=False, sep=',').dropna(axis=1)
                df_resample = resample(df, '2S')
                df_resample = df_resample['t_avg_plan_time']     
                dfs.append(df_resample) # Get list of dataframes from CSV file paths

            losses = pd.concat([losses, pd.DataFrame({name+str(idx) :df_resample})], axis=1)

        avg_losses = pd.concat([avg_losses, pd.DataFrame({name :losses.mean(axis=1)})], axis=1)
    return avg_losses