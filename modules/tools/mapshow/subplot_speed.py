from map import Map

class SpeedSubplot:
    def __init__(self, ax):
        self.ax = ax
        self.speed_lines = []
        self.speed_lines_size = 3
        colors = ['b', 'g', 'r', 'k']
        for i in range(self.speed_lines_size):
            line, = ax.plot(
                [0], [0],
                colors[i % len(colors)] + ".",
                lw=3 + i * 3,
                alpha=0.4)
            self.speed_lines.append(line)

        ax.set_xlabel("t (second)")
        ax.set_xlim([-2, 10])
        ax.set_ylim([-1, 10])
        ax.set_ylabel("speed (m/s)")

        self.set_visible(False)

    def set_visible(self, visible):
        for line in self.speed_lines:
            line.set_visible(visible)

    def show(self, planning):
        cnt = 0
        planning.speed_data_lock.acquire()
        for name in planning.speed_data_time.keys():
            if cnt >= self.speed_lines_size:
                print "WARNING: number of path lines is more than " \
                      + str(self.speed_lines_size)
                continue
            if len(planning.speed_data_time[name]) <= 1:
                continue
            speed_line = self.speed_lines[cnt]
            speed_line.set_visible(True)
            speed_line.set_xdata(planning.speed_data_time[name])
            speed_line.set_ydata(planning.speed_data_val[name])
            speed_line.set_label(name[0:5])
            cnt += 1

        self.ax.legend(loc="upper left", borderaxespad=0., ncol=5)
        #self.ax.axis('equal')
        planning.speed_data_lock.release()