#!/usr/bin/env python
###
# File Created: Wednesday, February 6th 2019, 9:05:13 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By: Charlene Leong
# Last Modified: Thursday, February 7th 2019, 6:33:25 pm
###
import os
from os.path import isfile, join, split
import glob
import pandas as pd


# Timeseries data
import datetime
import numpy as np
from pandas.tseries.frequencies import to_offset


def get_avg_losses():
    """Returns pandas df of average losses over all runs of all files 
        
    Returns:
        pd.df -- df of averages over all funs of all runs of files
    """
    # Returns pd df of fullpaths, paths and filenames
    files = files_to_df('final/*/*.csv')
    list_num_files('final')

    # Iterate over all runs for every unique filename,
    # resample df to 2S interval,
    # create losses df,
    # concat mean to avg loss df

    print('List of filenames:')
    print('---------------------------')
    avg_losses = pd.DataFrame()
    for fn in files['filename'].unique():
        name = fn.split('.')[0]
        print(fn)
        paths = files[files['filename'] == fn]['fullpath']

        losses = pd.DataFrame()
        benchmark_losses = pd.DataFrame()
        for idx, path in enumerate(paths):
            df = pd.read_csv(path, index_col=False, sep=',').dropna(axis=1)

            # If default or ompl, average t_avg_plan_time else loss
            if any(x in path for x in ['default', 'ompl']):
                benchmark_losses = pd.concat(
                    [benchmark_losses, all_benchmark_losses(df, name, idx)], axis=1)
                avg_benchmark_loss = get_avg_benchmark_loss(benchmark_losses)
                avg_loss = avg_benchmark_loss
            else:
                df_tseries = to_tseries(name, df, rate='2S')
                df_tseries = df_tseries['loss']
                losses = pd.concat([losses, pd.DataFrame(
                    {name+str(idx): df_tseries})], axis=1)
                avg_loss = pd.DataFrame({name: losses.mean(axis=1)})

        avg_losses = pd.concat([avg_losses, avg_loss], axis=1)
    return avg_losses


def all_benchmark_losses(df, name, idx):
    benchmark_losses = pd.DataFrame()
    for planner in df['planner']:
        if planner == 'KPIECEkConfigDefault':
            continue
        col_name = name + "_" + planner.split('kConfigDefault')[0]
        benchmark = float(df[(df['planner'] == planner)]['t_avg_plan_time'])
        benchmark_df = pd.DataFrame(
            {col_name+str(idx): benchmark}, index=pd.to_timedelta(np.arange(7200), unit='s'))
        benchmark_df = to_tseries(name, benchmark_df, rate='2S')
        benchmark_losses = pd.concat([benchmark_losses, benchmark_df], axis=1)
    return benchmark_losses


def get_avg_benchmark_loss(benchmark_losses):
    cols = set([c[:-1] for c in benchmark_losses.columns])
    avg_benchmark_loss = pd.DataFrame()
    for c_unique in cols:
        planner_cols = [c for c in benchmark_losses.columns if c_unique in c]
        planner_df = benchmark_losses[planner_cols]
        col_name = planner_df.columns[0][:-1]
        avg_benchmark_loss = pd.concat([avg_benchmark_loss, pd.DataFrame(
            {col_name: planner_df.mean(axis=1)})], axis=1)
    return avg_benchmark_loss


def files_to_df(dir):
    """Returns pd df of fullpaths, paths and filenames
    
    Arguments:
        dir {str} -- Desired root dir, can be wildcard
    
    Returns:
        pd.df     -- DF of fullpaths, paths and filenames
    """
    files = pd.DataFrame(
        [file for file in glob.glob(dir)], columns=["fullpath"])
    #    fullpath
    # 0  dir\*\*.csv
    # 1  dir\*\*.csv
    # ..
    files_split = files['fullpath'].str.rsplit(
        os.sep, 1, expand=True).rename(columns={0: 'path', 1: 'filename'})
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
        print(d + ': ' + str(num_files[idx]) + ' files')
    print('\n')


def to_tseries(name, df, rate='2S'):
    if any(x in name for x in ['default', 'ompl']):
        df_tseries = df.resample(rate).sum()/int(rate[0])
    else:
        # Resetting index to elapsed time 7200s
        # Setting first value to 0
        df.loc[-1] = df.loc[0]
        df.index = df.index + 1
        df = df.sort_index()
        df.loc[0, 'elapsed_time'] = 0
        df.loc[0, 'n_trial'] = 0

        # Setting last value to 7200
        df = df.loc[df['elapsed_time'] <= 7200]
        df = df.append(df.iloc[-1]).reset_index(drop=True)
        df.iloc[-1, df.columns.get_loc('elapsed_time')] = 7200
        df.iloc[-1, df.columns.get_loc('n_trial')
                ] = df.iloc[-2, df.columns.get_loc('n_trial')]+1

        # Converting to timedelta and setting time as index
        df['elapsed_time'] = df['elapsed_time'].apply(
            lambda x: datetime.timedelta(seconds=x))
        df = df.set_index('elapsed_time')
        df_tseries = resample(df, rate)

    return df_tseries


# Linear Resampling irregular timeseries data
def resample(tseries, rate='T', short_rate='S', max_gap=None):
    """Resample (unevenly spaced) timeseries data linearly by first upsampling to a
        high frequency (short_rate) then downsampling to the desired rate.
    
    Arguments:
        tseries {pd.df} -- a panda timeseries df
    
    Keyword Arguments:
        rate {pd freq string} -- rate that tseries should be resampled to (default: {'T'})
        short_rate {pd freq string} -- intermediate upsampling rate (default: {'S'})
        max_gap {pd freq string} --null intervals larger than `max_gap` are being treated as missing
        data and not interpolated. if None, always interpolate. must be provided as pandas
        frequency string format, e.g. '6h' (default: {None})
    
    Returns:
        tseries {pd.df}  -- resampled tseries df
    """
    # return series if empty
    if tseries.empty:
        return tseries

    # check for timedelta index
    assert isinstance(
        tseries.index[0], pd.Timedelta), 'Object must have a datetime-like index.'

    # sort tseries by time
    tseries.sort_index(inplace=True)

    # create timedelta from frequency string
    rate_delta = to_offset(rate).delta

    # compute time intervals
    diff = np.diff(tseries.index) / np.timedelta64(1, 's')

    if max_gap is not None:
        # identify intervals in tseries larger than max_gap
        idx = np.where(np.greater(diff, to_offset(
            max_gap).delta.total_seconds()))[0]
        start = tseries.index[idx].tolist()
        stop = tseries.index[idx + 1].tolist()
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
    tseries = tseries.resample(short_rate).mean().interpolate()

    # downsample to desired rate
    tseries = tseries.resample(rate).ffill()

    # replace values in large gap itervals with NaN
    if max_gap is not None:
        for start, stop in big_gaps:
            tseries[start:stop] = None

    return tseries
