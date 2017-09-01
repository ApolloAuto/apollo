import rospy
from modules.planning.proto import planning_pb2
import matplotlib.pyplot as plt
from planning import Planning
import matplotlib.animation as animation
from subplot_st_main import StMainSubplot
from subplot_path import PathSubplot
from subplot_sl_main import SlMainSubplot
from subplot_speed import SpeedSubplot
from localization import Localization
import argparse

planning = Planning()
localization = Localization()

def update(frame_number):
    #st_main_subplot.show(planning)
    #st_speed_subplot.show(planning)
    map_path_subplot.show(planning, localization)
    dp_st_main_subplot.show(planning)
    qp_st_main_subplot.show(planning)
    speed_subplot.show(planning)
    sl_main_subplot.show(planning)

def planning_callback(planning_pb):
    planning.update_planning_pb(planning_pb)
    localization.update_localization_pb(
        planning_pb.debug.planning_data.adc_position)

    planning.compute_st_data()
    planning.compute_sl_data()
    planning.compute_path_data()
    planning.compute_speed_data()

def add_listener():
    rospy.init_node('st_plot', anonymous=True)
    rospy.Subscriber('/apollo/planning', planning_pb2.ADCTrajectory,
                     planning_callback)

def press_key(event):
    if event.key == '+' or event.key == '=':
        map_path_subplot.zoom_in()
    if event.key == '-' or event.key == '_':
        map_path_subplot.zoom_out()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="plot_planning is a tool to display "
                    "planning trajs on a map.",
        prog="plot_planning_old.py")
    parser.add_argument(
        "-m",
        "--map",
        action="store",
        type=str,
        required=False,
        default=None,
        help="Specify the map file in txt or binary format")
    args = parser.parse_args()

    add_listener()
    fig = plt.figure()
    fig.canvas.mpl_connect('key_press_event', press_key)

    ax = plt.subplot2grid((3, 3), (0, 0), rowspan=2, colspan=2)
    map_path_subplot = PathSubplot(ax, args.map)

    ax1 = plt.subplot2grid((3, 3), (0, 2))
    speed_subplot = SpeedSubplot(ax1)

    ax2 = plt.subplot2grid((3, 3), (1, 2))
    dp_st_main_subplot = StMainSubplot(ax2, 'QpSplineStSpeedOptimizer')

    ax3 = plt.subplot2grid((3, 3), (2, 2))
    qp_st_main_subplot = StMainSubplot(ax3, 'DpStSpeedOptimizer')

    ax4 = plt.subplot2grid((3, 3), (2, 0), colspan=2)
    sl_main_subplot = SlMainSubplot(ax4)

    ani = animation.FuncAnimation(fig, update, interval=100)

    ax.axis('equal')
    plt.show()
