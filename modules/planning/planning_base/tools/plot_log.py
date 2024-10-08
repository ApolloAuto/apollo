#!/usr/bin/python3.6
# encoding=utf-8
"""
python tools/plot_log.py -f /apollo/data/log/planning.INFO -t 11:50:34
"""

import argparse
import os
import sys
import time
import datetime
import matplotlib.pyplot as plt
import log_util
from matplotlib.widgets import Button
from matplotlib.gridspec import GridSpec
from matplotlib import ticker
import json


def plot_frame(fig, ax, lines, line_st_num, line_ed_num):
    """plot ref frame"""


if __name__ == '__main__':
    global g_argv
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', action='store', dest='log_config_path',
                        required=True, help='log config path')
    parser.add_argument('-f', action='store', dest='log_file_path',
                        required=True, help='log file path')
    parser.add_argument('-t', action='store', dest='time',
                        required=False, help='time begin to search')
    parser.add_argument('-ut', action='store', dest='unix_time',
                        required=False, help='unix time begin to search')
    parser.add_argument('-s', action='store', dest='seq',
                        required=False, help='sequence number to search')
    g_argv = parser.parse_args()
    print('Please wait for loading data...')
    print(os.getcwd())
    config = {}
    with open(g_argv.log_config_path) as fp:
        config = json.load(fp)
    # load log file
    print(g_argv)
    file_path = g_argv.log_file_path
    search_time = g_argv.time
    search_seq = g_argv.seq
    unix_time = g_argv.unix_time
    input = open(file_path, 'r')
    lines = input.readlines()
    line_search_num = 0
    if search_time:
        line_search_num = log_util.search_time_line(lines, search_time)
    elif search_seq:
        line_search_num = log_util.search_seq_line(lines, search_seq)
    elif unix_time:
        search_time = datetime.utcfromtimestamp(
            float(unix_time)).strftime('%H:%M:%S')
        line_search_num = log_util.search_time_line(lines, search_time)
        print("from unixtime %s to data time %s" % (unix_time, search_time))
    else:
        print('search all file')
    print("line_search_num:", line_search_num)
    seq_id = 0
    if line_search_num == 0:
        line_st_num = 0
        line_ed_num = len(lines)
    else:
        line_st_num, line_ed_num, seq_id = log_util.search_current(
            lines, line_search_num)
    if line_st_num < 0 or line_ed_num < 0:
        print('[ERROR] search reach last line, may reach last frame, quit!')
        sys.exit(0)
    row = config["row"]
    col = config["col"]
    fig, axes = plt.subplots(row, col, figsize=[9, 15])
    # fig = plt.figure(figsize=[9, 15])
    # gs = GridSpec(1, 1, figure=fig)
    ax = {}
    for item in config["subplot"]:
        print(item)
        key = item["title"]
        if row == 1 and col == 1:
            ax[key] = axes
        elif row == 1:
            ax[key] = axes[item["col"]]
        elif col == 1:
            ax[key] = axes[item["row"]]
        else:
            ax[key] = axes[item["col"]][item["row"]]
    callback = log_util.Index(fig, ax, line_st_num,
                              line_ed_num, lines, config,file_path)
    callback.plot_frame("frame:" + str(seq_id))
    prev1frame = plt.axes([0.2, 0.01, 0.1, 0.05])
    prev10frame = plt.axes([0.3, 0.01, 0.1, 0.05])
    next1frame = plt.axes([0.4, 0.01, 0.1, 0.05])
    next10frame = plt.axes([0.5, 0.01, 0.1, 0.05])
    exitframe = plt.axes([0.6, 0.01, 0.1, 0.05])
    bprev1 = Button(prev1frame, '-1')
    bprev1.on_clicked(callback.prev1)
    bprev10 = Button(prev10frame, '-10')
    bprev10.on_clicked(callback.prev10)
    bnext1 = Button(next1frame, '+1')
    bnext1.on_clicked(callback.next1)
    bnext10 = Button(next10frame, '+10')
    bnext10.on_clicked(callback.next10)
    bexit = Button(exitframe, 'exit')
    bexit.on_clicked(callback.exit)
    plt.gca().xaxis.set_major_formatter(ticker.FormatStrFormatter('%.6f'))
    plt.gca().yaxis.set_major_formatter(ticker.FormatStrFormatter('%.6f'))
    plt.show()
    plt.ion()
