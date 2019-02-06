#!/usr/bin/env python
###
# File Created: Wednesday, February 6th 2019, 9:06:21 pm
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

from files import files_to_df, list_num_files

# Linear Resampling irregular timeseries data
def resample(df, rate='T', short_rate='S', max_gap=None):
    """Resample (unevenly spaced) timeseries data linearly by first upsampling to a
        high frequency (short_rate) then downsampling to the desired rate.
    
    Arguments:
        df {pd.df} -- a panda timeseries df
    
    Keyword Arguments:
        rate {pd freq string} -- rate that df should be resampled to (default: {'T'})
        short_rate {pd freq string} -- intermediate upsampling rate (default: {'S'})
        max_gap {pd freq string} --null intervals larger than `max_gap` are being treated as missing
        data and not interpolated. if None, always interpolate. must be provided as pandas
        frequency string format, e.g. '6h' (default: {None})
    
    Returns:
        df -- resampled df
    """
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
    avg_losses = pd.DataFrame()
    print('List of filenames:')
    print('---------------------------')
    for fn in files['filename'].unique():
        name = fn.split('.')[0]
        print(name)
        
        paths = files[files['filename'] == fn]['fullpath']
        losses = pd.DataFrame()
        for idx, path in enumerate(paths):
            df = pd.read_csv(path, index_col=False, sep=',').dropna(axis=1)
            df_resample = resample(df, '2S')

            # If default or ompl, average t_avg_plan_time else loss
            if any(x in path for x in ['default', 'ompl']):
                df_resample = df_resample['t_avg_plan_time']     
            else:
                df_resample = df_resample['loss']     
 
            losses = pd.concat([losses, pd.DataFrame({name+str(idx): df_resample})], axis=1)
        avg_losses = pd.concat([avg_losses, pd.DataFrame({name: losses.mean(axis=1)})], axis=1)
    
    return avg_losses

def speedup_factor_plot(run, planner_select, planner, benchmark='default', mode_select=None, figsize=[15,10]):
    """Plots speedup factor plot of given run, planner select and planner
    
    Arguments:
        run {int} -- run number
        planner_select {str} -- planner select
        planner {str} -- selected planner
    
    Keyword Arguments:
        benchmark {str} -- ompl or default (default: {'default'})
        mode_select {list[modes]} -- list of desired modes (default: {None})
        figsize {list[x, y]} -- Figsize
    """
    
    # Fig settings
    plt.rcParams['figure.figsize'] = figsize
    plt.rcParams['legend.loc'] = 'upper right'
    fig,ax = plt.subplots(1)
    cm = plt.get_cmap('jet')
    ax.set_prop_cycle('color', [cm(i) for i in np.linspace(0, 1, 10)])

    fps = [file for file in glob.glob(join('final', 'run_'+str(run), '*.csv'))]
    dir, fn = split(fps[0])
    
    # Extracting ompl and defaults
    default_fn = planner_select+'_default.csv'
    default_df = pd.read_csv(join(dir, default_fn), index_col=False)
    default = float(default_df[(default_df['planner'].str.contains(planner))]['t_avg_plan_time'])
    
    ompl_fn = planner_select+'_ompl.csv'
    ompl_df = pd.read_csv(join(dir, ompl_fn), index_col=False)
    ompl = float(ompl_df[(ompl_df['planner'].str.contains(planner))]['t_avg_plan_time'])

    # Setting benchmark
    if benchmark == 'default':
        benchmark = default
    elif benchmark == 'ompl':
        benchmark = ompl

    # Plotting ompl and default
    elapsed_time = np.linspace(0, 7200, 500)
    default_df = pd.DataFrame({'elapsed_time': elapsed_time, 'default': default})
    default_df['default'] = benchmark/default_df['default']

    ax.plot('elapsed_time', 'default', data=default_df, label='default')
    ompl_df = pd.DataFrame({'elapsed_time': elapsed_time, 'ompl': ompl})
    ompl_df['ompl'] = benchmark/ompl_df['ompl']
    ax.plot('elapsed_time', 'ompl', data=ompl_df, label='ompl')
        
    # Planner select filepaths
    for fp in fps:
        if planner_select in fp:
            if not any(x in fp for x in ['default', 'ompl']):
                # fp = <planner_select>_<mode>_<planner>.csv
                dir, fn = split(fp)
                fn = fn.split('_')
                planner_fn = fn[-1].split('.')[-2]
                mode = fn[2]
                
                if (mode_select != None) and (mode not in mode_select):
                    fps.remove(fp)
                else:
                    # Filtering fns for selected planner
                    if re.search(planner_fn, planner, re.IGNORECASE): 
                        # print(fp)
                        mode_df = pd.read_csv(fp, index_col=False)
                        # Speed up factor
                        max_speed_up = benchmark
                        speed_up = list(mode_df['loss'])
                        for idx, loss in enumerate(speed_up):
                            if loss < max_speed_up:
                                max_speed_up = loss
                            else:
                                speed_up[idx] = max_speed_up

                        # Plotting speedup factor plot                                            
                        mode_df = pd.DataFrame({'elapsed_time': mode_df['elapsed_time'], mode: speed_up})
                        mode_df[mode+'_factor'] = benchmark/mode_df[mode]
                        ax.plot('elapsed_time', mode+'_factor', data=mode_df, label=mode)

    plt.title('Run ' + str(run) + ': '+ planner_select + ' ' + planner)
    plt.xticks([0, 1440, 2880, 4320, 5760, 7200])
    plt.xlabel('Time spent exploring solutions (secs)')
    plt.ylabel('Speed up over default')
    plt.legend()

def avg_speedup_factor_plot(avg_losses, planner_select, planner, benchmark='default', mode_select=None, figsize=[15,10]):
    """Plots average speedup factor plot over all runs of given planner select and planner
    
    Arguments:
        avg_losses {pd.df} -- pd.df of avg losses
        planner_select {str} -- planner select
        planner {str} -- selected planner
    
    Keyword Arguments:
        benchmark {str} -- ompl or default (default: {'default'})
        mode_select {list[modes]} -- list of desired modes (default: {None})
        figsize {list[x, y]} -- Figsize
    """
    
    plt.rcParams['figure.figsize'] = figsize
    
    # Getting planner select df from avg losses df 
    df = avg_losses.copy()
    def_cols = [planner_select+'_default', planner_select+'_ompl']
    cols = [c for c in df.columns if planner_select in c if planner in c]
    cols = sorted(cols)
    cols = def_cols + cols
    ps_df = df[cols]
    
    # Renaming planner select df cols
    legend = [c.split('_')[2] for c in cols]
    ps_df.columns = legend
    
    # Setting benchmark
    if benchmark == 'default':
        benchmark = float(ps_df['default'][0])
    elif benchmark == 'ompl':
        benchmark = float(ps_df['ompl'][0])
    
    # Setting modes
    modes = legend
    if (mode_select != None):
        modes = ['default', 'ompl']
        modes = modes + mode_select
        
    # Creating speedup df of all modes
    speedup_df = pd.DataFrame({'default': benchmark/ps_df['default'], 'ompl': benchmark/ps_df['ompl']})
    for mode in modes:
        if not any(x in mode for x in ['default', 'ompl']): 
            speedup = list(ps_df[mode])
            max_speedup = benchmark
            for idx, loss in enumerate(speedup):
                if loss < max_speedup:
                    max_speedup = loss
                else:
                    speedup[idx] = max_speedup

            mode_df = pd.Series(speedup, index=ps_df[mode].index, name=mode)
            mode_df = benchmark/mode_df
            speedup_df = pd.concat([speedup_df, mode_df], axis=1)
            
    # Convert TimedeltaIndex to seconds and setting xticks
    speedup_df.index = speedup_df.index.seconds
    xtick_vals = [0, 1440, 2880, 4320, 5760, 7200] 
    # Plot
    ax = speedup_df.plot(title=planner_select+' '+planner, colormap='jet', xticks=xtick_vals)
    ax.set_xlabel('Time spent exploring solutions (secs)')
    ax.set_ylabel('Speed up over default')
    ax.legend(loc='upper right')