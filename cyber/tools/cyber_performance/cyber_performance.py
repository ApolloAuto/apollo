#!/usr/bin/env python3
# ****************************************************************************
# Copyright 2024 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************

import time
import json
import os
import re
import threading
import platform
import subprocess
from queue import Queue
from datetime import datetime
from flask import Flask, jsonify, render_template_string
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from cyber.python.cyber_py3 import cyber

app = Flask(__name__)
data_history = {
    "time": Queue(maxsize = 20),
    "data": {},
}
today = datetime.now()
data_lock = threading.Lock()
if 'APOLLO_ENV_WORKROOT' in os.environ:
    DIRECTORY_TO_WATCH = os.path.join(os.environ['APOLLO_ENV_WORKROOT'], "dumps")
    PERFORMANCE_TO_WRITE = os.path.join(os.environ['APOLLO_ENV_WORKROOT'], "data")
else:
    DIRECTORY_TO_WATCH = os.path.join("/apollo", "dumps")
    PERFORMANCE_TO_WRITE = os.path.join("/apollo", "data")
if not os.path.exists(DIRECTORY_TO_WATCH):
    os.mkdir(DIRECTORY_TO_WATCH)
if not os.path.exists(PERFORMANCE_TO_WRITE):
    os.mkdir(PERFORMANCE_TO_WRITE)
dumps_fio = open(
  os.path.join(PERFORMANCE_TO_WRITE,
    "performance_dumps.{}.json".format(today.strftime("%m-%d-%Y"))), "a+")
sample_times = 0
response = {}
update_time = 5000
chasis_channle = "/apollo/canbus/chassis"
iface_path = "/sys/class/net"
autodrive = False
machine = platform.machine()
nethogs_buf = ""
process_network_io = {}
network_data_lock = threading.Lock()

sudo_prv = subprocess.run(
    ['sudo', '-n', 'true'], stdout=None, stderr=None).returncode == 0

remove_list = []
for i in os.listdir(DIRECTORY_TO_WATCH):
    if i.startswith("performance_dumps"):
        date_str = i.split(".")[1]
        try:
            file_save_date = datetime.strptime(date_str, "%m-%d-%Y")
        except:
            remove_list.append(os.path.join(DIRECTORY_TO_WATCH, i))
            continue 
        if (today - file_save_date).days > 30:
            remove_list.append(os.path.join(DIRECTORY_TO_WATCH, i))
for i in remove_list:
    os.remove(i)

cyber_instance = None
debounce_period = 1
last_modified = {}

class CyberChannelecho(object):

    def __init__(self, channel_name):
        cyber.init()
        self.pattern = r"driving_mode:\s*(\w+)"
        self.channel_name = channel_name
        self.node = cyber.Node("listener_node_echo")
        self.node.create_rawdata_reader(channel_name, self.callback)

    def callback(self, raw_data):
        global autodrive
        msgtype = cyber.ChannelUtils.get_msgtype(self.channel_name, 0).decode('utf-8')
        text = cyber.ChannelUtils.get_debugstring_rawmsgdata(msgtype, raw_data).decode('utf-8')
        match = re.search(self.pattern, text)
        if match:
            ret = match.group(1)
            if ret == "COMPLETE_AUTO_DRIVE":
                autodrive = True
            else:
                autodrive = False 

def cyber_task():
    global cyber_instance
    cyber_instance = CyberChannelecho(chasis_channle)

def watcher_run():
    global DIRECTORY_TO_WATCH
    event_handler = Handler()
    observer = Observer()
    
    observer.schedule(event_handler, DIRECTORY_TO_WATCH)
    observer.start()
    try:
        while True:
            time.sleep(5)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()

def gpu_memory_usage(pid):
    gpu_mem_kernel_file = "/sys/kernel/debug/nvmap/iovmm/maps"
    gpu_mem = 0

    if pid == -1:
        return gpu_mem
    if machine == "aarch64":
        total, processes = read_process_table(gpu_mem_kernel_file)
        if total is not None:
            for p in processes:
                if pid == int(p[0]):
                    gpu_mem = int(p[3])
                    break
    else:
        query_cmd = "nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader,nounits"
        p = subprocess.run(query_cmd, shell=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout = p.stdout.decode("utf-8").strip().splitlines()
        for line in stdout:
            eles = [i.strip() for i in line.split(",")]
            if len(eles) < 2:
                break
            if int(eles[0]) == pid:
                gpu_mem = int(eles[1])
                break
    return gpu_mem

def read_process_table(path_table):
    """
    This method list all processes working with GPU

    ========== ============ ======== =============
    user       process      PID      size
    ========== ============ ======== =============
    user       name process number   dictionary
    ========== ============ ======== =============

    :return: list of all processes
    :type spin: list
    """
    if not os.path.exists(path_table):
        return None, None
    if not os.access(path_table, os.R_OK):
        return None, None
    MEM_TABLE_REG = re.compile(r'^(?P<user>\w+)\s+(?P<process>[^ ]+)\s+(?P<PID>\d+)\s+(?P<size>\d+)(?P<unit>\w)\n')
    TOT_TABLE_REG = re.compile(r'total\s+(?P<size>\d+)(?P<unit>\w)')

    table = []
    total = {}
    with open(path_table, "r") as fp:
        for line in fp:
            # Search line
            match = re.search(MEM_TABLE_REG, line)
            if match:
                parsed_line = match.groupdict()
                data = [
                    parsed_line['PID'],
                    parsed_line['user'],
                    parsed_line['process'],
                    int(parsed_line['size']),
                ]
                table += [data]
                continue
            # Find total on table
            match = re.search(TOT_TABLE_REG, line)
            if match:
                parsed_line = match.groupdict()
                total = int(parsed_line['size'])
                continue
    # return total and table
    return total, table

class Handler(FileSystemEventHandler):
    @staticmethod
    def on_any_event(event):
        global data_history 
        global data_lock
        global autodrive
        global network_data_lock, process_network_io

        if event.is_directory:
            return None

        elif event.event_type == 'created' or event.event_type == 'modified':
            if not event.src_path.endswith(".system.data"):
                return None

            current_time = time.time()
            last_mod_time = last_modified.get(event.src_path, 0)

            if current_time - last_mod_time <= debounce_period:
                return None
            else:
                last_modified[event.src_path] = current_time
            
            print(f'Event type: {event.event_type}  path : {event.src_path}')
            file_name = event.src_path.replace(DIRECTORY_TO_WATCH + "/", "")
            process_name = file_name.replace(".system.data", "")
            pid_file_name = event.src_path.replace(".system.data", ".data")
            latency_file_name = event.src_path.replace(".system.data", ".latency.data") 
            while True:
                with open(event.src_path, 'r') as f:
                    current_data = {}
                    current_data["autodrive"] = autodrive
                    contents = f.read().split("\n")
                    if len(contents) == 0:
                        continue
                    elif len(contents) == 1 and contents[0] == "":
                        continue
                    for line in contents:
                        if line == "":
                            continue
                        instance = line.strip().split(" : ")
                        if len(instance) == 1:
                            continue
                        
                        if instance[0].endswith("cpu_usage"):
                            current_data["BASIC - cpu_usage(%, single-core)"] = float(instance[1]) * 100
                        elif instance[0].endswith("memory_resident"):
                            current_data["BASIC - memory(MB)"] = int(instance[1]) / 1024 / 1024
                        elif instance[0].endswith("disk_read_bytes_second"):
                            current_data["BLOCK_DEVICE_IO - block_device_io_read(rkB/s)"] = int(instance[1]) / 1024
                        elif instance[0].endswith("disk_write_bytes_second"):
                            current_data["BLOCK_DEVICE_IO - block_device_io_write(wkB/s)"] = int(instance[1]) / 1024
                        else:
                            continue
                    break
            pid = -1

            with open(pid_file_name, "r") as pf:
                contents = pf.readlines()
                for line in contents:
                    if line == "":
                        continue
                    instance = line.strip().split(" : ")
                    if len(instance) == 1:
                        continue
                    if instance[0].endswith("_pid"):
                        pid = int(instance[1])
                        break
            gpu_mem = gpu_memory_usage(pid)

            with open(latency_file_name, "r") as lf:
                contents = lf.readlines()
                has_proc_info = False
                for line in contents:
                    if "proc_latency :" in line:
                        instance = line.split(" : ")
                        latency_name = instance[0].replace("mainboard_", "")
                        latency_val = float((instance[1].strip())) / 1000
                    else:
                        continue
                    has_proc_info = True
                    current_data[f"E2E_LATENCY - {latency_name}(ms)"] = latency_val
            if not has_proc_info:
                return 
            with network_data_lock:
                if pid in process_network_io:
                    current_data["ETHERNET_DEVICE_IO - ethernet_device_io_write(wkB/s)"] = process_network_io[pid]["tx"]
                    current_data["ETHERNET_DEVICE_IO - ethernet_device_io_read(rkB/s)"] = process_network_io[pid]["rx"] 
                else:
                    current_data["ETHERNET_DEVICE_IO - ethernet_device_io_write(wkB/s)"] = 0
                    current_data["ETHERNET_DEVICE_IO - ethernet_device_io_read(rkB/s)"] = 0  
            
            current_data["BASIC - gpu_memory(MB)"] = gpu_mem / 1024
                    
            with data_lock:
                if process_name not in data_history["data"]:  
                    data_history["data"][process_name] = {}
                    data_history["data"][process_name]["time"] = Queue(maxsize = 20)
                
                for key in current_data:
                    if key not in data_history["data"][process_name]:
                        data_history["data"][process_name][key] = Queue(maxsize = 20)

                time_format = datetime.now().strftime("%m/%d/%Y, %H:%M:%S")
                if data_history["data"][process_name]["time"].full():
                    data_history["data"][process_name]["time"].get()
                data_history["data"][process_name]["time"].put(time_format)

                for key, value in current_data.items():
                    if data_history["data"][process_name][key].full():
                        data_history["data"][process_name][key].get()
                    data_history["data"][process_name][key].put(value)


def sample_task():
  global data_history 
  global sample_times
  global dumps_fio
  global response
  global autodrive

  p = subprocess.run(
      "which tegrastats", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

  tegrastats = None
  if p.returncode == 0:
      tegrastats = p.stdout.decode("utf-8").strip()

  system_info = {
    "system-cpu": Queue(maxsize = 20),
    "system-memory": Queue(maxsize = 20),
    "system-gpu": Queue(maxsize = 20),
    "time": Queue(maxsize = 20),
    "system-io-usage": Queue(maxsize = 20),
    "autodrive": Queue(maxsize = 20),
    "block-device-io" : {},
    "ethernet-device-io": {}
  }
  
  while True:
      sample_times = sample_times + 1
      system_cpu = None
      system_memory = None
      system_gpu = None
      system_io_usage = None
      if tegrastats is not None:
          time_format = datetime.now().strftime("%m/%d/%Y, %H:%M:%S")
          p = subprocess.run(
              " ".join(["timeout", "2", tegrastats]), shell=True, 
              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
          stdout = p.stdout.decode("utf-8").strip()

          cpu_usage_pattern = re.compile(r'\bCPU \[(.*?)\]')
          match_result = cpu_usage_pattern.findall(stdout)
          if len(match_result) > 0:
              usages = re.findall(r'(\d+)%@', match_result[0])
              system_cpu = 0
              for usage in usages:
                  system_cpu += float(usage)
              system_cpu = system_cpu / len(usages)
          
          gpu_usage_pattern = re.compile(r'GR3D_FREQ (\d+)%')
          match_result = gpu_usage_pattern.findall(stdout)
          if len(match_result) > 0:
              system_gpu = float(match_result[0])

          p = subprocess.run(
              "vmstat 1 2 | awk '{print $16}' | tail -n 1", shell=True, 
              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
          stdout = p.stdout.decode("utf-8").strip()
          system_io_usage = float(stdout)
      else:
          p = subprocess.run(
              "top -bn2 | grep Cpu | awk '{print $8}' | tail -n 1", shell=True, 
              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
          stdout = p.stdout.decode("utf-8").strip()
          time_format = datetime.now().strftime("%m/%d/%Y, %H:%M:%S")
          system_cpu = 100 - float(stdout) 

          system_gpu = 0 

          p = subprocess.run(
              "vmstat | awk '{print $16}' | tail -n 1", shell=True, 
              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
          stdout = p.stdout.decode("utf-8").strip()
          system_io_usage = float(stdout)
      p = subprocess.run("cat /proc/meminfo",
          shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      stdout = p.stdout.decode("utf-8").strip().splitlines()
      meminfo = {}
      for i in stdout:
          k, v = i.split(":")[0].strip().lower(), i.split(":")[1].strip()
          value = round(int(v.strip().split(' ')[0]) / 1024, 2)
          meminfo[k] = value
      
      total = meminfo["memtotal"]
      buffers = meminfo["buffers"]
      cached = meminfo["cached"]
      sreclaimable = meminfo["sreclaimable"]
      free = meminfo["memfree"]
      shm = meminfo["shmem"]
      if "memavailable" not in meminfo:
          used_without_shm = total - buffers - cached - sreclaimable
          used = used_without_shm + shm
      else:
          available = meminfo["memavailable"]
          used = total - available
      system_memory = used 
          
      if system_cpu is not None \
            and system_gpu is not None \
            and system_memory is not None \
            and system_io_usage is not None:
          if system_info["time"].full():
              system_info["time"].get()
          system_info["time"].put(time_format) 
          if system_info["system-cpu"].full():
              system_info["system-cpu"].get()
          system_info["system-cpu"].put(system_cpu) 
          if system_info["system-gpu"].full():
              system_info["system-gpu"].get()
          system_info["system-gpu"].put(system_gpu)
          if system_info["system-memory"].full():
              system_info["system-memory"].get()
          system_info["system-memory"].put(system_memory)
          if system_info["system-io-usage"].full():
              system_info["system-io-usage"].get()
          system_info["system-io-usage"].put(system_io_usage)
          if system_info["autodrive"].full():
              system_info["autodrive"].get()
          system_info["autodrive"].put(autodrive) 
      
      p = subprocess.run(
          "lsblk -o NAME,TYPE,SIZE,TRAN --json", shell=True, 
          stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      stdout = p.stdout.decode("utf-8").strip()
      blk_devices = json.loads(stdout)
      blk_monitor_devices = []
      try:
          for dev in blk_devices["blockdevices"]:
              if dev["type"] != "disk":
                  continue
                
              if dev["tran"] == "nvme" or dev["tran"] == "sata": 
                  blk_monitor_devices.append(dev["name"])
              elif dev["tran"] == "null" and "children" in dev:
                  blk_monitor_devices.append(dev["name"])
          blk_monitor_devices = list(filter(
              lambda x: os.path.exists(f"/dev/{x}"), blk_monitor_devices))
      except Exception as ex:
          blk_monitor_devices = []

      if len(blk_monitor_devices) > 0:
          for dev in blk_monitor_devices:
              if dev not in system_info["block-device-io"]:
                  system_info["block-device-io"][dev] = {
                      "rkB/s": Queue(maxsize = 20),
                      "wkB/s": Queue(maxsize = 20),
                      "r/s": Queue(maxsize = 20),
                      "w/s": Queue(maxsize = 20),
                      "r_await": Queue(maxsize = 20),
                      "w_await": Queue(maxsize = 20),
                      "aqu-sz": Queue(maxsize = 20)
                  }

          query_devices = " ".join(blk_monitor_devices)
          stat_length = len(blk_monitor_devices) + 1
          query_cmd = f"iostat -xk 1 2 {query_devices} | grep -v '^$' | tail -n {stat_length}"

          p = subprocess.run(query_cmd, shell=True, 
              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
          stdout = p.stdout.decode("utf-8").strip().splitlines()
          
          monitor_items = list(filter(lambda x: x != "", stdout[0].split(" ")))
          for i in range(1, len(stdout)):
              dev_res = list(filter(lambda x: x != "", stdout[i].split(" ")))
              dev_name = dev_res[0]
              for j in range(1, len(monitor_items)):
                  if monitor_items[j] not in system_info["block-device-io"][dev_name]:
                      continue
                  monitor_item = monitor_items[j]
                  if system_info["block-device-io"][dev_name][monitor_item].full():
                      system_info["block-device-io"][dev_name][monitor_item].get()
                  system_info["block-device-io"][dev_name][monitor_item].put(dev_res[j])
      
      eth_devices = []
      for i in os.listdir(iface_path):
          if os.path.exists(os.path.join(iface_path, i, "device", "driver")):
              eth_devices.append(i)
      if len(eth_devices) > 0:
          for dev in eth_devices:
              if dev not in system_info["ethernet-device-io"]:
                  system_info["ethernet-device-io"][dev] = {
                      "rxkB/s": Queue(maxsize = 20),
                      "txkB/s": Queue(maxsize = 20),
                      "%ifutil": Queue(maxsize = 20)
                  }
          query_cmd = "sar -n DEV 1 1 | grep --color=never Average"
          p = subprocess.run(query_cmd, shell=True, 
              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
          stdout = p.stdout.decode("utf-8").strip().splitlines()

          monitor_items = list(filter(lambda x: x != "", stdout[0].split(" ")))
          for i in range(1, len(stdout)):
              dev_res = list(filter(lambda x: x != "", stdout[i].split(" ")))
              dev_name = dev_res[1]
              if dev_name not in system_info["ethernet-device-io"]:
                  continue
              for j in range(2, len(monitor_items)):
                  if monitor_items[j] not in system_info["ethernet-device-io"][dev_name]:
                      continue
                  monitor_item = monitor_items[j]
                  if system_info["ethernet-device-io"][dev_name][monitor_item].full():
                      system_info["ethernet-device-io"][dev_name][monitor_item].get()
                  system_info["ethernet-device-io"][dev_name][monitor_item].put(dev_res[j]) 

      # system metrics
      response = {
          "data": {"system": {}}
      }

      # status
      response["data"]["system"]["time"] = list(system_info["time"].queue)
      response["data"]["system"]["autodrive"] = list(system_info["autodrive"].queue)
      
      # basic performance metrics
      response["data"]["system"]["BASIC - system - cpu_usage(%, all-core)"] = list(system_info["system-cpu"].queue)
      response["data"]["system"]["BASIC - system - gpu_usage(%)"] = list(system_info["system-gpu"].queue)
      response["data"]["system"]["BASIC - system - memory(MB)"] = list(system_info["system-memory"].queue)

      # io metrics
      response["data"]["system"]["BLOCK_DEVICE_IO - system - io_wait_usage(%)"] = list(system_info["system-io-usage"].queue)
      for dev in system_info["block-device-io"]:
          for io_metric in system_info["block-device-io"][dev]:
              response["data"]["system"][f"BLOCK_DEVICE_IO - {dev} - {io_metric}"] = \
                  list(system_info["block-device-io"][dev][io_metric].queue)
      for dev in system_info["ethernet-device-io"]:
          for io_metric in system_info["ethernet-device-io"][dev]:
              response["data"]["system"][f"ETHERNET_DEVICE_IO - {dev} - {io_metric}"] = \
                  list(system_info["ethernet-device-io"][dev][io_metric].queue)
    
      # process metrics
      for p in data_history["data"]:
          response["data"][p] = {}
          for monitor_instance in data_history["data"][p]:
              response["data"][p][monitor_instance] = list(
                  data_history["data"][p][monitor_instance].queue)

      if sample_times >= 20:
          dumps_fio.write(json.dumps(response))
          dumps_fio.write('\n')
          dumps_fio.flush()
          sample_times = 0

      time.sleep(update_time / 1000)
@app.route('/get_data', methods=['GET'])  
def get_data():  
  global response 
  
  return jsonify(response)
  
@app.route('/')  
def index():  
    return render_template_string('''
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>cyber_performance</title>
  <!-- Include Chart.js -->
  <script src="https://apollo-system.cdn.bcebos.com/archive/9.0/chart.js"></script>
  <style>
    .chart-container {
      display: flex;
      flex-wrap: wrap;
      justify-content: flex-start; /* Align charts to the start of the container */
    }

    .chart-item {
      width: calc(33.33% - 20px); /* Adjusted to make each chart take up one-third of the container width with some spacing */
      margin: 10px; /* Add margin for spacing between charts */
      max-width: calc(33.33% - 20px); /* Set max-width to ensure charts don't exceed one-third of the container width */
    }

    canvas {
      max-width: 100%;
    }
  </style>
</head>
<body>
  <select id="processSelector">
        <!-- dynamically added by JS code -->
  </select>
  <div class="chart-container" id="chartsContainer"></div>
  

  <script>
    let selectedProcess = 'system';
    let chartMap = {};

    // Function to update the process selection dropdown
    function updateProcessSelection(processNames) {
      const processSelector = document.getElementById('processSelector');
      processSelector.innerHTML = ''; // Should we clear current option?
      processNames.forEach(processName => {
        const option = document.createElement('option');
        option.value = processName;
        option.textContent = processName;
        if (processName === selectedProcess) {
          option.selected = true;
        }
        processSelector.appendChild(option);
      });
    }

    // Function to handle process selection changes
    function onProcessSelectionChange() {
      const processSelector = document.getElementById('processSelector');
      if (selectedProcess !== processSelector.value) {
        selectedProcess = processSelector.value;
        const chartsContainer = document.getElementById('chartsContainer');
        chartsContainer.innerHTML = ''; // Remove all child elements (charts)
        chartMap = {}; // Reset the chartMap since all charts are removed
      }
      updateCharts();
    }

    async function updateCharts() {
      const data = await fetchData();
      if (!data || !data.data) {
        return;
      }
      const time = data.time;
      const processData = data.data;

      const processNames = Object.keys(processData);

      const chartsContainer = document.getElementById('chartsContainer');

      processNames.forEach(processName => {
        const monitoringItems = Object.keys(processData[processName]);
        if (processName === selectedProcess) { 
          // Only proceed if process is selected
          // Existing code to create/update charts
          monitoringItems.filter(item => item !== 'time' && item !== 'autodrive').forEach(item => {
            const chartId = processName + '-' + item + 'Chart';

            if (chartMap[chartId]) {
              // Update existing chart
              const chart = chartMap[chartId];
              chart.data.labels = processData[processName]['time'];
              chart.data.datasets[0].data = processData[processName][item];
              chart.update();
            } else {
              // Create new chart container and canvas
              const chartItem = document.createElement('div');
              chartItem.classList.add('chart-item');
              chartsContainer.appendChild(chartItem);

              const canvas = document.createElement('canvas');
              canvas.id = chartId;
              chartItem.appendChild(canvas);

              // Initialize chart data object
              const chartData = {
                labels: processData[processName]['time'],
                datasets: [{
                  label: item,
                  data: processData[processName][item],
                  borderColor: getRandomColor(),
                  backgroundColor: 'rgba(255, 255, 255, 0)', // Transparent background
                  borderWidth: 1
                }]
              };

              // Create new chart and save reference
              const ctx = canvas.getContext('2d');
              const newChart = new Chart(ctx, {
                type: 'line',
                data: chartData,
                options: {
                  scales: {
                    xAxes: [{
                      type: 'category', // Set type to 'category' for string labels
                      position: 'bottom',
                      ticks: {
                        stepSize: 1,
                        beginAtZero: true
                      }
                    }],
                    yAxes: [{
                      ticks: {
                        beginAtZero: true
                      }
                    }]
                  },
                  plugins: [{
                    afterDraw: function(chart) {
                      const ctx = chart.ctx;
                      ctx.save();
                      ctx.textAlign = 'center';
                      ctx.textBaseline = 'middle';
                      ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
                      ctx.font = '12px Arial';
                      ctx.fillText('Memory (MB)', chart.chartArea.right + 20, chart.chartArea.top + (chart.chartArea.bottom - chart.chartArea.top) / 2);
                      ctx.restore();
                    }
                  }]
                }
              });

              chartMap[chartId] = newChart;
            }
          });
        }

      });
    }

    document.getElementById('processSelector').addEventListener('change', onProcessSelectionChange);

    // Function to fetch data from backend API
    async function fetchData() {
      const response = await fetch('/get_data');
      const data = await response.json();
      if (data && data.data) {
        const processNames = Object.keys(data.data);
        updateProcessSelection(processNames);
      }
      return data;
    }

    // Function to generate random color
    function getRandomColor() {
      const letters = '0123456789ABCDEF';
      let color = '#';
      for (let i = 0; i < 6; i++) {
        color += letters[Math.floor(Math.random() * 16)];
      }
      return color;
    }

    // Update charts every 5 seconds
    setInterval(updateCharts, 5000);

    // Initial call to update charts
    updateCharts();
  </script>
</body>
</html>
    ''')  

if __name__ == '__main__':
    system_sample_thread = threading.Thread(target=sample_task)
    system_sample_thread.daemon = True
    system_sample_thread.start()
    
    watchdog_thread = threading.Thread(target=watcher_run)
    watchdog_thread.daemon = True
    watchdog_thread.start()

    if sudo_prv:
        nethogs_process = subprocess.Popen(
            ['sudo', 'nethogs', '-t'], stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, bufsize=1, universal_newlines=True)

        def parse_nethogs_output():
            global nethogs_buf, process_network_io, network_data_lock
            for line in iter(nethogs_process.stdout.readline, ''):
                if line.startswith("Refreshing:"):
                    with network_data_lock:
                        process_network_io = {}
                        nethogs_buf_list = list(filter(lambda x: x!= "", nethogs_buf.split("\n")))
                        for i in nethogs_buf_list:
                            raw_line = list(filter(lambda x: x != "", i.split(" ")))
                            raw_process_info = raw_line[0].split("\t")
                            process_info = raw_process_info[0].split("/")
                            if len(process_info) < 3 or int(process_info[-2]) == 0:
                                continue
                            process_network_io[int(process_info[-2])] = {} 
                            process_network_io[int(process_info[-2])]["rx"] = float(raw_process_info[2])
                            process_network_io[int(process_info[-2])]["tx"] = float(raw_process_info[1])
                    nethogs_buf = ""
                else:
                    nethogs_buf = nethogs_buf + line
        
        network_sample_thread = threading.Thread(target=parse_nethogs_output)
        network_sample_thread.daemon = True
        network_sample_thread.start()

    cyber_task()
    
    app.run(host="0.0.0.0", threaded=True)
