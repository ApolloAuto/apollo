import numpy as np
import argparse
from decimal import Decimal

traj_len = 30

def cal_diff_avg(dt, gt):
    """
    calculate the average error of trajectory points.
    
    Args:
        dt (list): the prediction trajectory.
        gt (list): the ground truth trajectory.
    
    Returns:
        tuple: reutrn the average error and number of points.
    """
    sum = 0
    point_num = 0
    if (len(dt) != traj_len or len(gt) != traj_len):
        print('ERROR!!', 'len of dt: ', len(dt), 'len of gt: ', len(gt))
        return 0
    for i in range(traj_len):
        if (gt[i][0] != 0 and gt[i][1] != 0):
            point_num += 1
            point_error = (dt[i][0] - gt[i][0]) ** 2 + (dt[i][1] - gt[i][1]) ** 2
            point_error = point_error ** 0.5
            sum += point_error
    return [sum, point_num]


def cal_diff_final(dt, gt):
    """
    calculate the final point error.
    
    Args:
        dt (list): the prediction trajectory.
        gt (list): the ground truth trajectory.
    
    Returns:
        List[float]: return the final point error and whether the final point exist.
    """
    final_point_error = 0
    is_final_point_exist = 0
    if (gt[-1][0] != 0 and gt[-1][1] != 0):
        is_final_point_exist = 1
        final_point_error = (dt[-1][0] - gt[-1][0]) ** 2 + (dt[-1][1] - gt[-1][1]) ** 2
        final_point_error = final_point_error ** 0.5
    return [final_point_error, is_final_point_exist]


def log_parser(file_path):
    """
    This function is used to parse the log file and calculate the ADE, FDE and Miss Rate of the trajectory.
    
    Args:
        file_path (str): The path of the log file.
    
    Returns:
        None.
    """
    # all the object
    all_obj = dict()
    
    # process the log file
    with open(file_path) as f:
        lines = f.readlines()

        for line in lines:

            line = line.strip()

            if ('prediction_eval_log (traj)' in line):

                prediction_eval_log = line.split('(traj): ')
                curr_pred_info = prediction_eval_log[1].split(', ')

                timestamp = curr_pred_info[0][:12]
                # if int(curr_pred_info[0][12:13]) > 5:
                #     timestamp = str(Decimal(timestamp) + Decimal('0.1'))
                id_key = curr_pred_info[1]

                if 'traj' not in all_obj[id_key][timestamp].keys():
                    all_obj[id_key][timestamp]['traj'] = []
                if (len(all_obj[id_key][timestamp]['traj']) < 30):
                    # timestamp, id, pred_x, pred_y
                    all_obj[id_key][timestamp]['traj'].append([float(curr_pred_info[2]), float(curr_pred_info[3])])

            if ('prediction_eval_log (info)' in line):

                prediction_eval_log = line.split('(info): ')
                curr_info = prediction_eval_log[1].split(', ')
                
                # timestamp, id, priority, type, x, y
                obj_info_curr = [curr_info[2], curr_info[3], curr_info[4], curr_info[5]]
                
                # take only one decimal place
                timestamp = curr_info[0][:12]
                # if int(curr_info[0][12:13]) > 5:
                #     timestamp = str(Decimal(timestamp) + Decimal('0.1'))
                id_key = curr_info[1]
                if id_key not in all_obj.keys():
                    all_obj[id_key] = {}
                if timestamp not in all_obj[id_key].keys():
                    all_obj[id_key][timestamp] = {}
                all_obj[id_key][timestamp]['info'] = obj_info_curr
    
    # Begin to calculate

    ADE = 0
    FDE = 0
    traj_miss_num = 0
    total_point_num = 0
    totol_fde_num = 0

    for key in all_obj:
        gt = []

        timestamp_obj = []
        for value in all_obj[key].keys():
            timestamp_obj.append(value)

        timestamp = timestamp_obj[0]
        while float(timestamp) <= float(timestamp_obj[-1]):
            # Maintain a actual trajectory with a length of 30
            if(len(gt) >= 30):
                gt = gt[1:]

            if timestamp in all_obj[key]:
                gt.append([float(all_obj[key][timestamp]['info'][2]),
                        float(all_obj[key][timestamp]['info'][3])])
            else:
                # If obstacle is not detected, it will be temporarily filled with 0 but will not participate in the calculation
                gt.append([0, 0])

            if (len(gt) == 30):
                t_pred_begin = str(Decimal(timestamp) - Decimal('3.0'))
                if (t_pred_begin in all_obj[key] and 'traj' in all_obj[key][t_pred_begin]):
                    # 0: UNKNOWN, 1: UNKNOWN_MOVABLE, 2: UNKNOWN_UNMOVABLE
                    # 3: PEDESTRIAN, 4: BICYCLE, 5: VEHICLE
                    if (all_obj[key][t_pred_begin]['info'][1] == '5'):
                        # 1: CAUTION, 2: NORMAL, 3: IGNORE
                        # if (all_obj[key][t_pred_begin]['info'][0] == '1'):
                        dt = all_obj[key][t_pred_begin]['traj']
                        diff, sample_point_num = cal_diff_avg(dt, gt)
                        ADE += diff
                        total_point_num += sample_point_num
                        diff_final, is_final_exist = cal_diff_final(dt, gt)
                        if (diff_final > 2.0):
                            traj_miss_num += 1
                        FDE += diff_final
                        totol_fde_num += is_final_exist
            timestamp = str(Decimal(timestamp) + Decimal('0.1'))
    print('total_point_num: ', total_point_num)
    print('total_fde_num: ', totol_fde_num)
    print('total_miss_num: ', traj_miss_num)
    print('ADE: ', ADE / total_point_num)
    print('FDE: ', FDE / totol_fde_num)
    print('Miss Rate: ', traj_miss_num / totol_fde_num)

if __name__== "__main__" :
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--file-path',
        type=str,
        required=True,
        help='The prediction log file path.')
    args = parser.parse_args()

    log_parser(args.file_path)
