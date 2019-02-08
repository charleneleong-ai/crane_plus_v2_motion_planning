#!/usr/bin/env python
###
# File Created: Wednesday, February 6th 2019, 9:06:21 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By: Charlene Leong
# Last Modified: Friday, February 8th 2019, 11:38:17 am
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

    fps = [file for file in glob.glob(join('output', 'final', 'run_'+str(run), '*.csv'))]
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

def avg_speedup_factor_plot(avg_losses, planner_select, planner, benchmark='ompl', mode_select=None, figsize=[15,10]):
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
    
    # Getting planner select df from avg losses df, ordering cols with default and ompl first
    df = avg_losses.copy()
    cols = [c for c in df.columns if planner_select in c if planner in c]
    for c in cols:
        if any(x in c for x in ['default', 'ompl']):
            cols.remove(c)
    cols = sorted(cols)
    cols = [planner_select+'_default_'+planner, planner_select+'_ompl_'+planner] + cols
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