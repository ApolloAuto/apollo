import rospy
from modules.planning.proto import planning_pb2
import matplotlib.pyplot as plt
from planning import Planning
import matplotlib.animation as animation
from subplot_traj_speed import TrajSpeedSubplot
from subplot_traj_acc import TrajAccSubplot
from subplot_traj_path import TrajPathSubplot

planning = Planning()

def update(frame_number):
    traj_speed_subplot.show(planning)
    traj_acc_subplot.show(planning)
    traj_path_subplot.show(planning)

def planning_callback(planning_pb):
    planning.update_planning_pb(planning_pb)
    planning.compute_traj_data()

def add_listener():
    rospy.init_node('st_plot', anonymous=True)
    rospy.Subscriber('/apollo/planning', planning_pb2.ADCTrajectory,
                     planning_callback)

def press_key():
    pass

if __name__ == '__main__':
    add_listener()
    fig = plt.figure(figsize=(14, 6))
    fig.canvas.mpl_connect('key_press_event', press_key)

    ax = plt.subplot2grid((2, 2), (0, 0))
    traj_speed_subplot = TrajSpeedSubplot(ax)

    ax2 = plt.subplot2grid((2, 2), (0, 1))
    traj_acc_subplot = TrajAccSubplot(ax2)

    ax3 = plt.subplot2grid((2, 2), (1, 0))
    traj_path_subplot = TrajPathSubplot(ax3)

    ani = animation.FuncAnimation(fig, update, interval=100)

    plt.show()
