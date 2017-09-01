import rospy
from modules.planning.proto import planning_pb2
import matplotlib.pyplot as plt
from planning import Planning
import matplotlib.animation as animation
from subplot_st_main import StMainSubplot
from subplot_st_speed import StSpeedSubplot

planning = Planning()

def update(frame_number):
    st_main_subplot.show(planning)
    st_speed_subplot.show(planning)

def planning_callback(planning_pb):
    planning.update_planning_pb(planning_pb)
    planning.compute_st_data()

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

    ax = plt.subplot2grid((1, 2), (0, 0))
    st_main_subplot = StMainSubplot(ax, 'QpSplineStSpeedOptimizer')

    ax2 = plt.subplot2grid((1, 2), (0, 1))
    st_speed_subplot = StSpeedSubplot(ax2, "QpSplineStSpeedOptimizer")

    ani = animation.FuncAnimation(fig, update, interval=100)

    plt.show()
