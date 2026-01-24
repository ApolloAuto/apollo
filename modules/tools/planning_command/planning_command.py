import modules.common_msgs.planning_msgs.planning_command_pb2 as planning_command_pb2
import modules.common_msgs.external_command_msgs.free_space_command_pb2 as free_space_command_pb2
import modules.common_msgs.external_command_msgs.command_status_pb2 as command_status_pb2
import modules.common_msgs.external_command_msgs.action_command_pb2 as action_pb2
import modules.common_msgs.external_command_msgs.precise_parking_command_pb2 as precise_parking_command_pb2
import modules.common_msgs.external_command_msgs.valet_parking_command_pb2 as parking_pb2
import modules.common_msgs.external_command_msgs.zone_cover_command_pb2 as zone_cover_command_pb2
import google.protobuf.any_pb2 as any_pb2
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
import modules.tools.common.proto_utils as proto_utils
import matplotlib.pyplot as plt
from matplotlib import ticker
seq_num = 1
# bdkjy_reverse_map
polygon_free_space = [(437578.6, 4432555.6),
                      (437577.24, 4432552.8),
                      (437574.3, 4432550.0),
                      (437579.8, 4432544.7),
                      (437584.10, 4432551.24)
                      ]
# bdkjy_reverse_map_parking_out
polygon_parking_out = [(437547.8, 4432544.35),
                       (437548.3, 4432541.9),
                       (437549.2, 4432537.6),
                       (437564.3, 4432540.7),
                       (437562.9, 4432547.3)]
precise_parking = [437560.83, 4432539.28, 1.7]
# bdkjy_map
# polygon = [(437556.24, 4432546.77),
#            (437557.5, 4432543.0),
#            (437563.8, 4432543.87),
#            (437564.36, 4432540.45),
#            (437572.07, 4432541.35),
#            (437570.94, 4432548.4),
#            (437556.24, 4432546.77)]
pose = {}

# 1: parking out point
pose['1'] = [437558.5, 4432545.12, 0.16]
# 2: free space1
pose['2'] = [437579.68, 4432549.63, 2.55]
# 2: free space2
pose['3'] = [437578.36, 4432552.67, -2.16]


def add_header(planning_command):
    global seq_num
    planning_command.header.sequence_num = seq_num
    planning_command.header.timestamp_sec = cyber_time.Time.now().to_sec()
    planning_command.header.module_name = "planning_command"
    seq_num = seq_num + 1


def set_polygon(fsc, polygon):
    roi = fsc.drivable_roi
    for pt in polygon:
        pt_pb = roi.point.add()
        pt_pb.x = pt[0]
        pt_pb.y = pt[1]


def set_pose(fsc, pose):
    fsc.parking_spot_pose.x = pose[0]
    fsc.parking_spot_pose.y = pose[1]
    fsc.parking_spot_pose.heading = pose[2]


if __name__ == '__main__':
    x = []
    y = []
    for pt in polygon_free_space:
        x.append(pt[0])
        y.append(pt[1])
    print(x)
    print(y)
    plt.plot(x, y, '*-')
    plt.gca().xaxis.set_major_formatter(ticker.FormatStrFormatter('%.6f'))
    plt.gca().yaxis.set_major_formatter(ticker.FormatStrFormatter('%.6f'))
    # plt.show()
    cyber.init()
    node = cyber.Node("planning_command")
    writer = node.create_writer(
        "/apollo/planning_command", planning_command_pb2.PlanningCommand)
    client_free_space = node.create_client(
        "/apollo/external_command/free_space", free_space_command_pb2.FreeSpaceCommand, command_status_pb2.CommandStatus)
    client_pull_over = node.create_client(
        "/apollo/external_command/action", action_pb2.ActionCommand, command_status_pb2.CommandStatus)
    client_parking = node.create_client(
        "/apollo/external_command/valet_parking", parking_pb2.ValetParkingCommand, command_status_pb2.CommandStatus)
    client_precise_parking = node.create_client(
        "/apollo/external_command/precise_parking", precise_parking_command_pb2.PreciseParkingCommand, command_status_pb2.CommandStatus)
    client_zone_cover = node.create_client(
        "/apollo/external_command/zone_cover", zone_cover_command_pb2.ZoneCoverCommand, command_status_pb2.CommandStatus)
    while not cyber.is_shutdown():
        m = input(
            "0:lanefollow 1: pullover 2: parking 3:parking out 4:freespace 1 5: freespace 2 6: precise_parking 7: zone_cover\n")
        planning_command = planning_command_pb2.PlanningCommand()
        add_header(planning_command)
        print(m)
        if m == '0':
            act = action_pb2.ActionCommand()
            add_header(act)
            act.command = action_pb2.ActionCommandType.FOLLOW
            client_pull_over.send_request(act)
        elif m == "1":
            act = action_pb2.ActionCommand()
            add_header(act)
            act.command = action_pb2.ActionCommandType.PULL_OVER
            client_pull_over.send_request(act)
        elif m == "2":
            park_name = input("please input parking spot id:")
            parking = parking_pb2.ValetParkingCommand()
            add_header(parking)
            parking.parking_spot_id = park_name
            print(client_parking.send_request(parking))
        elif m == "3":  # parking out
            fsc = free_space_command_pb2.FreeSpaceCommand()
            add_header(fsc)
            set_polygon(fsc, polygon_parking_out)
            set_pose(fsc, pose['1'])
            print(fsc)
            client_free_space.send_request(fsc)
        elif m == "4":  # free space pose
            fsc = free_space_command_pb2.FreeSpaceCommand()
            add_header(fsc)
            set_polygon(fsc, polygon_free_space)
            set_pose(fsc, pose['2'])
            print(fsc)
            client_free_space.send_request(fsc)
        elif m == "5":  # free space pose
            fsc = free_space_command_pb2.FreeSpaceCommand()
            add_header(fsc)
            set_polygon(fsc, polygon_free_space)
            set_pose(fsc, pose['3'])
            print(fsc)
            client_free_space.send_request(fsc)
        elif m == "6":  # free space pose
            fsc = precise_parking_command_pb2.PreciseParkingCommand()
            x = float(input("please input precise_parking x:"))
            y = float(input("please input precise_parking y:"))
            heading = float(input("please input precise_parking heading:"))
            inwards = bool(input("please input precise_parking inwards:"))
            add_header(fsc)
            # set_pose(fsc, precise_parking)
            set_pose(fsc, [x, y, heading])
            fsc.mission_type = precise_parking_command_pb2.PreciseMissionType.DUMP
            # fsc.parking_inwards = False
            fsc.parking_inwards = inwards
            print(fsc)
            client_precise_parking.send_request(fsc)

            # msg = any_pb2.Any()
            # msg.Pack(fsc, str(fsc))
            # planning_command.custom_command.CopyFrom(msg)
            # writer.write(planning_command)
        elif m == "7":  # zone_cover
            fsc = zone_cover_command_pb2.ZoneCoverCommand()
            area_id = input("please input area_id: ")
            add_header(fsc)
            fsc.zone_cover_area_id = area_id
            print(fsc)
            client_zone_cover.send_request(fsc)

    cyber.shutdown()
