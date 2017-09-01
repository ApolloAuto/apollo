from planning import Planning
from modules.planning.proto import planning_pb2
from modules.planning.proto import planning_config_pb2

class StSpeedSubplot:

    def __init__(self, ax, st_name):
        self.speed_limit_line = ax.plot([0], [0], "r-",
                                        lw=6, alpha=0.5, label="speed limits")[0]
        self.speed_line = ax.plot([0], [0], "k-",
                                  lw=3, alpha=0.5, label="speed planned")[0]
        self.speed_upper_bound_line = \
            ax.plot([0], [0], "b-", lw=1, alpha=1, label="speed upper bound")[0]
        self.speed_lower_bound_line = \
            ax.plot([0], [0], "b-", lw=3, alpha=1, label="speed lower bound")[0]
        self.st_name = st_name
        ax.set_xlim(-10, 70)
        ax.set_ylim(-1, 15)
        ax.set_xlabel("s (m)")
        ax.set_ylabel("v (m/s)")
        ax.set_title("SV graph")
        ax.legend(loc="upper left", bbox_to_anchor=(0, 1), ncol=2, borderaxespad=0.)
        self.set_visible(False)

    def set_visible(self, visible):
        self.speed_limit_line.set_visible(visible)
        self.speed_line.set_visible(visible)
        self.speed_upper_bound_line.set_visible(visible)
        self.speed_lower_bound_line.set_visible(visible)

    def show(self, planning):
        self.set_visible(False)

        planning.st_data_lock.acquire()
        if self.st_name not in planning.st_curve_s:
            planning.st_data_lock.release()
            return
        planned_speed_s = planning.st_curve_s[self.st_name]
        planned_speed_v = planning.st_curve_v[self.st_name]
        self.speed_line.set_xdata(planned_speed_s)
        self.speed_line.set_ydata(planned_speed_v)
        self.speed_line.set_visible(True)

        self.speed_limit_line.set_xdata(planning.st_speed_limit_s[self.st_name])
        self.speed_limit_line.set_ydata(planning.st_speed_limit_v[self.st_name])
        self.speed_limit_line.set_visible(True)

        self.speed_upper_bound_line.set_xdata(planning.st_speed_constraint_s[self.st_name])
        self.speed_upper_bound_line.set_ydata(planning.st_speed_constraint_upper[self.st_name])
        self.speed_upper_bound_line.set_visible(True)

        self.speed_lower_bound_line.set_xdata(planning.st_speed_constraint_s[self.st_name])
        self.speed_lower_bound_line.set_ydata(planning.st_speed_constraint_lower[self.st_name])
        self.speed_lower_bound_line.set_visible(True)

        planning.st_data_lock.release()



