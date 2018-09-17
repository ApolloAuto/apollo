import sys
import argparse
from collections import defaultdict
from collections import OrderedDict

# transport perf list
trans_event_map = defaultdict(list)
sched_event_map = defaultdict(list)

# record cur_id for routine
sched_fusion_map = {}

# notify event should be deel specially
notify_tag_map = {}

event_trace_map = OrderedDict()
split_str = "**_**"

class event():
    def __init__(self):
        self.pre_id = None
        self.id_index = {"5" : -1, "1" : -1, "3" : -1, "2" : -1, "4" : -1}

class trans_event():
    def __init__(self):
        self.stamp = None
        self.id = None

class sched_event():
    def __init__(self):
        self.id = None
        self.pre_id = None
        self.pre_stamp = None
        self.pre_id_index = None
        self.stamp = None
        self.notify_stamp = None

def perf_data_process(input_file):
    global event, event_trace
    with open(input_file) as f:
        for line in f:
            line=line.strip('\n')
            items = line.split()

            if int(items[0]) == 1:
                event_type = items[0]
                event_id = items[1]
                channel_name = items[2]
                seq_id = items[3]
                stamp = items[4]
                event_trace_map[stamp] = line
                identify_id = channel_name + split_str + seq_id
                e = trans_event()
                e.stamp = stamp
                e.id = event_id
                trans_event_map[identify_id].append(e)

            elif int(items[0]) == 0:
                event_type = items[0]
                event_id = items[1]
                rt_name = items[2]
                proc_id = items[3]
                notify_stamp = items[4]
                t_start = items[5]
                stamp = items[6]

                if not sched_fusion_map.has_key(rt_name):
                    e_v = event()
                    sched_fusion_map[rt_name] = e_v;
                if not notify_tag_map.has_key(rt_name):
                    notify_tag_map[rt_name] = False

                event_trace_map[stamp] = line

                identify_id = rt_name + split_str + event_id
                e = sched_event()

                if event_id == '4' and notify_tag_map[rt_name]:
                    continue
                if event_id == '4' and not notify_tag_map[rt_name]:
                    notify_tag_map[rt_name] = True

                if event_id == '5':
                    notify_tag_map[rt_name] = False

                if sched_fusion_map[rt_name].pre_id == None:
                    sched_fusion_map[rt_name].pre_id = event_id
                    sched_fusion_map[rt_name].id_index[event_id] += 1
                    continue

                e.id = event_id
                e.pre_id = sched_fusion_map[rt_name].pre_id
                e.pre_id_index = sched_fusion_map[rt_name].id_index[e.pre_id]
                e.stamp = stamp
                e.notify_stamp = notify_stamp
                sched_event_map[identify_id].append(e)
                sched_fusion_map[rt_name].id_index[event_id] += 1
                sched_fusion_map[rt_name].pre_id = event_id
            else:
                event_trace_map[items[0]] = line

    for identify_id in trans_event_map.keys():
        trans_events = trans_event_map[identify_id]
        pre_stamp = -1
        pre_id = -1
        for event in trans_events:
            if pre_id != -1 and event.id != '1' and pre_id != '3':
                latency = long(event.stamp) - long(pre_stamp)
                event_trace_map[event.stamp] = event_trace_map[event.stamp] + "\t" + pre_id + "\t" + pre_stamp  + "\t" + str(latency)
            pre_id = event.id
            pre_stamp = event.stamp

    for identify_id in sched_event_map.keys() :
        event_list = sched_event_map[identify_id]
        for index in range(0, len(event_list)):
            event = event_list[index]
            if not event.pre_id:
                continue
            pre_id = event.pre_id
            pre_id_index = event.pre_id_index
            rt_name = identify_id.split(split_str)[0]
            pre_identify_id = rt_name + split_str + pre_id
            pre_id_list = sched_event_map[pre_identify_id]
            if event.id == "1" or event.id == "4":
                continue
            else:
                event_pre = pre_id_list[pre_id_index]
                latency = long(event.stamp) - long(event_pre.stamp)
            event_trace_map[event.stamp] = event_trace_map[event.stamp] + "\t" + event.pre_id + "\t" + event_pre.stamp  + "\t" + str(latency)

    with open(input_file + '.opt', 'w') as pfile:
        for value in event_trace_map.values():
            pfile.write(value + "\n")


def args_parse():
    parse = argparse.ArgumentParser()
    parse.add_argument("input_file")
    parse.add_argument('-c', help = 'require one arguement, input_file: the processed perf file')
    args = parse.parse_args()
    return args

def main():
    args = args_parse()
    if args.input_file:
        perf_data_process(args.input_file)
if __name__ == '__main__':
    main()
