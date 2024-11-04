#!/usr/bin/python3.6
# encoding=utf-8
"""
python tools/plot_planning_perf.py -f /apollo/data/log/planning.INFO
"""

import argparse
import os
import sys
import time
import datetime
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.gridspec import GridSpec
from matplotlib import ticker
import matplotlib.dates as mdates
from datetime import datetime
import json


class FigPlot(object):
    """button callback function"""

    def __init__(self, fig, ax):
        self.ax = ax
        self.fig = fig

    def plot_frame(self, data):
        self.graphs = {}
        self.fig.suptitle("Plnning Perf")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("ms")
        self.ax.grid(True)
        self.ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S.%f'))
        start_t = None
        end_t = None
        for name in data:
            line, = self.ax.plot(data[name]["time"], data[name]["perf"], "-*", linewidth=2, label=name)
            self.graphs[name] = line
            start_t = data[name]['time'][0]
            end_t = data[name]['time'][-1]
        
        if start_t is not None and end_t is not None:
            self.ax.plot([start_t, end_t], [100, 100], "-*")

        legend = self.ax.legend()
        for legend_line in legend.get_lines():
            legend_line.set_picker(True)
            legend_line.set_pickradius(10)
            # if "set_aspect" in item:
            #     self.ax[key].set_aspect('equal')
        plt.connect('pick_event', self.on_pick)
        plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
        plt.draw()

    def on_pick(self, event):
        legend = event.artist
        isVisible = legend.get_visible()
        self.graphs[legend._label].set_visible(not isVisible)
        legend.set_visible(not isVisible)
        self.fig.canvas.draw()

def get_string_between(string, st, ed=''):
    """get string between keywords"""
    if string.find(st) < 0:
        return ''
    sub_string = string[string.find(st) + len(st):]
    if len(ed) == 0 or sub_string.find(ed) < 0:
        return sub_string.strip()
    return sub_string[:sub_string.find(ed)]


def get_planning_seq_num(line):
    """get planning seq num from line"""
    return get_string_between(line, 'start frame sequence id = [', ']')


def get_time(line):
    """get time from line"""
    return get_string_between(line, ' ', ' ')

def process_frame_data(data: dict, frame_lines):
    start_seq_time = None
    end_seq_time = None
    last_task_name = None
    current_task_name = None
    planning_name = None
    planning_perf = 0
    time1 = None
    time2 = None
    for line in frame_lines:
        if "Planning start frame sequence id" in line:
            start_seq_time = get_time(line)
            continue
        elif "Planning end frame sequence id" in line:
            end_seq_time = get_time(line)
            time1 = datetime.strptime(start_seq_time, "%H:%M:%S.%f")
            time2 = datetime.strptime(end_seq_time, "%H:%M:%S.%f")
            current_task_name = planning_name
            current_task_time_perf = planning_perf
        if start_seq_time is not None and "task name" in line:
            current_task_time = get_time(line)
            current_task_name = 'Task-' + get_string_between(line, 'task name [', '],')
            current_task_time_perf = float(get_string_between(line, '], ', ' ms'))
            if last_task_name is None:
                time1 = datetime.strptime(start_seq_time, "%H:%M:%S.%f")
            else:
                time1 = data[last_task_name]['time'][-1]
            time2 = datetime.strptime(current_task_time, "%H:%M:%S.%f")
        if start_seq_time is not None and "planning name" in line:
            planning_name = 'Plan-' + get_string_between(line, 'planning name [', '],')
            planning_perf = float(get_string_between(line, '], ', ' ms'))
            continue

        if not current_task_name in data.keys():
            data[current_task_name] = dict()
            data[current_task_name]['time'] = []
            data[current_task_name]['perf'] = []
        data[current_task_name]['time'].append(time1)
        data[current_task_name]['perf'].append(0)
        data[current_task_name]['time'].append(time1)
        data[current_task_name]['perf'].append(current_task_time_perf)
        data[current_task_name]['time'].append(time2)
        data[current_task_name]['perf'].append(current_task_time_perf)
        data[current_task_name]['time'].append(time2)
        data[current_task_name]['perf'].append(0)
        last_task_name = current_task_name
    
    if planning_perf > 90.0:
        return True
    else:
        return False

if __name__ == '__main__':
    global g_argv
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', action='store', dest='log_file_path',
                        required=True, help='log file path')
    # parser.add_argument('-t', action='store', dest='time',
    #                     required=False, help='time begin to search')
    # parser.add_argument('-s', action='store', dest='seq',
    #                     required=False, help='sequence number to search')
    g_argv = parser.parse_args()
    print('Please wait for loading data...')
    print(os.getcwd())
    config = {}
    # with open(g_argv.log_config_path) as fp:
    #     config = json.load(fp)
    # load log file
    file_path = g_argv.log_file_path
    input = open(file_path, 'r')
    lines = input.readlines()
    perf_log_file = file_path + "_perf_log"
    fout = open(perf_log_file, "w+")
    fout.write("This planning perf log was created on: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\n")
    fout.close()

    # 定义关键字
    keywords = ['Planning Perf', 'frame sequence id']
    data = {}
    find_sequence_start = False
    find_sequence_end = False
    frame_lines = []
    for line in lines:
        if "Planning start frame sequence id" in line:
            start_seq_id = get_string_between(line, 'sequence id = [', ']')
            find_sequence_start = True
            find_sequence_end = False
            frame_lines = []
            frame_lines.append(line)
        if "Planning Perf" in line:
            frame_lines.append(line)
        if "Planning end frame sequence id" in line:
            end_seq_id = get_string_between(line, 'sequence id = [', ']')
            find_sequence_end = True
            frame_lines.append(line)
        if find_sequence_start and find_sequence_end and start_seq_id == end_seq_id:
            is_perf_timeout = process_frame_data(data, frame_lines)
            find_sequence_start = False
            find_sequence_end = False
            if is_perf_timeout:
                fout = open(perf_log_file, "a+")
                for log_line in frame_lines:
                    fout.write(log_line)
                fout.close()
    
    print(f'Save planning performance timeout log to file: {perf_log_file}')
    
    fig, ax = plt.subplots(figsize=[10, 6])
    fig_plot = FigPlot(fig, ax)
    fig_plot.plot_frame(data)
    
    plt.show()
    plt.ion()
