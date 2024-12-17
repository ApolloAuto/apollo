from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
import modules.common_msgs.perception_msgs.perception_obstacle_pb2 as perception_obstacle_pb2
import threading
import time
import math
obs_poly = [[]] * 3
obs_poly[0] = [(437414.87645352894,4432519.946737605), (437420.7041803445,4432520.805670921),(437420.904429548,4432519.7323507555), (437415.23280138825,4432518.528933371)]
obs_poly[1] = [(437425.53023894905,4432521.440094629), (437429.9091883166,4432522.236959719), (437430.34108831757,4432520.733608818), (437425.9870466118,4432519.591352716)]
obs_poly[2] = [(437450.06361092144,4432525.741517296), (437455.1306381259,4432526.629585601), (437455.58288495895,4432524.2411506865), (437449.8351994263,4432523.395266685)]
# obs_poly[1] = [(751041.65, 2563971.9), (751044.12,
#                                         2563976.5), (751038.53, 2563984.9)]
# obs_poly = [[]]
# obs_poly[0] = [(751023.39, 2563972.72), (751034.12,
#                                          2563972.7), (751034.12, 2563970.4), (751023.39, 2563970.4)]

def generate_corner_coordinates(obs_polygon, obs_pb):
    x_min = 100000000
    x_max = -100000000
    y_min = 100000000
    y_max = -100000000
    for pt in obs_polygon:
        if pt[0] < x_min:
            x_min = pt[0]
        if pt[0] > x_max:
            x_max = pt[0]
        if pt[1] < y_min:
            y_min = pt[1]
        if pt[1] > y_max:
            y_max = pt[1]
        pt_pb = obs_pb.polygon_point.add()
        pt_pb.x = pt[0]
        pt_pb.y = pt[1]
    obs_pb.length = max(abs(x_max - x_min), abs(y_max - y_min))
    obs_pb.width = min(abs(x_max - x_min), abs(y_max - y_min))
    obs_pb.height = 1.5
    obs_pb.position.x = (x_max + x_min) / 2
    obs_pb.position.y = (y_max + y_min) / 2
    obs_pb.theta = 0.0
    obs_pb.velocity.x = 0.0
    obs_pb.velocity.y = 0.0


def add_obs(obs,id,msg_pb):
    obs_pb = msg_pb.perception_obstacle.add()
    generate_corner_coordinates(obs, obs_pb)
    obs_pb.type = perception_obstacle_pb2.PerceptionObstacle.PEDESTRIAN
    obs_pb.id = id

seq_num = 0


def add_header(msg):
    global seq_num
    msg.header.sequence_num = seq_num
    msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
    msg.header.module_name = "manual_perception"
    seq_num = seq_num + 1


def pub_func(writer):
    while not cyber.is_shutdown():
        global perception_msg
        add_header(perception_msg)
        writer.write(perception_msg)
        time.sleep(0.1)


perception_msg = None

if __name__ == '__main__':
    perception_msg = perception_obstacle_pb2.PerceptionObstacles()
    cyber.init()
    node = cyber.Node("manual_perception")
    writer = node.create_writer(
        "/apollo/perception/obstacles", perception_obstacle_pb2.PerceptionObstacles)
    thread = threading.Thread(target=pub_func, args=(writer,))
    thread.start()
    while not cyber.is_shutdown():
        m = input(
            "0: empty; 1: one obs; 2: two obs; 3: three obs\n")
        perception_msg.ClearField('perception_obstacle')
        print(m)
        if m == '1':
            add_obs(obs_poly[0],0,perception_msg)
        if m == '2':
          add_obs(obs_poly[0],0,perception_msg)
          add_obs(obs_poly[1],1,perception_msg)
        if m == '3':
          add_obs(obs_poly[0],0,perception_msg)
          add_obs(obs_poly[1],1,perception_msg)
          add_obs(obs_poly[2],2,perception_msg)

    cyber.shutdown()
