import sys
import matplotlib.pyplot as plt
from cyber_py.record import RecordReader
from modules.canbus.proto import chassis_pb2

def process(reader):
    last_steering_percentage = None
    last_speed_mps = None
    last_timestamp_sec = None
    speed_data = []
    d_steering_data = []

    for msg in reader.read_messages():
        if msg.topic == "/apollo/canbus/chassis":
            chassis = chassis_pb2.Chassis()
            chassis.ParseFromString(msg.message)

            steering_percentage = chassis.steering_percentage
            speed_mps = chassis.speed_mps
            timestamp_sec = chassis.header.timestamp_sec

            if chassis.driving_mode != chassis_pb2.Chassis.COMPLETE_AUTO_DRIVE:
                last_steering_percentage = steering_percentage
                last_speed_mps = speed_mps
                last_timestamp_sec = timestamp_sec               
                continue

            if last_timestamp_sec is None:
                last_steering_percentage = steering_percentage
                last_speed_mps = speed_mps
                last_timestamp_sec = timestamp_sec
                continue

            if (timestamp_sec - last_timestamp_sec) > 0.02:
                d_steering = (steering_percentage - last_steering_percentage) \
                    / (timestamp_sec - last_timestamp_sec)
                speed_data.append(speed_mps)
                d_steering_data.append(d_steering)

                last_steering_percentage = steering_percentage
                last_speed_mps = speed_mps
                last_timestamp_sec = timestamp_sec

    return speed_data, d_steering_data

if __name__ == "__main__":
    fns = sys.argv[1:]
    fig, ax = plt.subplots()

    for fn in fns:
        reader = RecordReader(fn)
        speed_data, d_steering_data = process(reader)
        ax.scatter(speed_data, d_steering_data)
    ax.set_xlim(-5, 40)
    ax.set_ylim(-300, 300)
    plt.show()    

