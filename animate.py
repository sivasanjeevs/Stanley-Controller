import sys
import os
sys.path.insert(0, os.getcwd())

import csv
import numpy as np

from math import radians
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation


from libs import CarDescription, DifferentialDriveModel, generate_cubic_spline # Changed import
from stanley_controller import StanleyController

class Simulation:

    def __init__(self):

        fps = 50.0

        self.dt = 1/fps
        self.map_size_x = 60
        self.map_size_y = 20
        self.frames = 1300
        self.loop = True

class Path:

    def __init__(self):

        # Get path to waypoints.csv
        with open('data/waypoints.csv', newline='') as f:
            rows = list(csv.reader(f, delimiter=','))

        x, y = [[float(i) for i in row] for row in zip(*rows[1:])]

        # Use the original waypoint polyline (no spline). Densify straight
        # segments for smoother target selection and heading computation.
        def densify_polyline(x_points, y_points, step=0.5):
            px_out = []
            py_out = []
            for i in range(len(x_points) - 1):
                x0, y0 = x_points[i], y_points[i]
                x1, y1 = x_points[i+1], y_points[i+1]
                dx, dy = x1 - x0, y1 - y0
                seg_len = (dx*dx + dy*dy) ** 0.7
                if seg_len == 0:
                    continue
                n = max(2, int(seg_len / step) + 1)
                t = np.linspace(0.0, 1.0, n, endpoint=False)
                px_out.extend(x0 + dx * t)
                py_out.extend(y0 + dy * t)
            px_out.append(x_points[-1])
            py_out.append(y_points[-1])
            return np.asarray(px_out), np.asarray(py_out)

        self.px, self.py = densify_polyline(x, y, step=0.5)
        dx = np.diff(self.px)
        dy = np.diff(self.py)
        if dx.size == 0:
            self.pyaw = np.array([0.0])
        else:
            self.pyaw = np.arctan2(dy, dx)
            self.pyaw = np.append(self.pyaw, self.pyaw[-1])

class Car:

    def __init__(self, init_x, init_y, init_yaw, sim_params, path_params):

        # Model parameters
        self.x = init_x
        self.y = init_y
        self.yaw = init_yaw
        self.v = 10.0
        self.delta = 0.0
        self.wheelbase = 2.5
        self.max_steer = radians(55)
        self.dt = sim_params.dt
        self.c_r = 0.01
        self.c_a = 2.0

        # Tracker parameters
        self.px = path_params.px
        self.py = path_params.py
        self.pyaw = path_params.pyaw
        self.k = 8.0
        self.ksoft = 1.0
        self.kyaw = 0.0
        self.ksteer = 0.0
        self.crosstrack_error = None
        self.target_id = None

        # Description parameters
        self.overall_length = 4.97
        self.overall_width = 1.964
        self.tyre_diameter = 0.4826
        self.tyre_width = 0.265
        self.axle_track = 1.7 # Use this for differential drive
        self.rear_overhang = (self.overall_length - self.wheelbase) / 2

        self.tracker = StanleyController(self.k, self.ksoft, self.kyaw, self.ksteer, self.max_steer, self.wheelbase, self.px, self.py, self.pyaw)
        self.kbm = DifferentialDriveModel(self.axle_track, self.dt) # Changed model initialization

    def drive(self):
        
        # Always compute control to keep CTE/yaw updated even when stopped
        v_L, v_R, self.target_id, self.crosstrack_error = self.tracker.stanley_control(self.x, self.y, self.yaw, self.v, self.delta, self.axle_track)

        # --- IMPROVED STOPPING LOGIC ---
        try:
            from math import hypot
            # Check if the target is the final waypoint
            is_last_target = self.target_id >= len(self.px) - 1
            # Calculate distance to the absolute final point
            distance_to_end = hypot(self.x - self.px[-1], self.y - self.py[-1])

            # If our target is the end and we are close enough, stop and zero displayed CTE
            if is_last_target and distance_to_end < 1.5: # Increased stopping radius to 1.5m
                self.v = 0.0
                self.delta = 0.0
                v_L = 0.0
                v_R = 0.0
                self.crosstrack_error = 0.0

        except Exception:
            pass

        # Only update the model if we are not stopped
        if self.v > 0.0:
            self.x, self.y, self.yaw = self.kbm.model(self.x, self.y, self.yaw, v_L, v_R) # Changed model update

class Fargs:

    def __init__(self, ax, sim, path, car, desc, outline, fr, rr, fl, rl, rear_axle, annotation, target, yaw_arr, yaw_data, crosstrack_arr, crosstrack_data, trail_line, trail_x, trail_y):
        
        self. ax = ax
        self.sim = sim
        self.path = path
        self.car = car
        self.desc = desc
        self.outline = outline
        self.fr = fr
        self.rr = rr
        self.fl = fl
        self.rl = rl
        self.rear_axle = rear_axle
        self.annotation = annotation
        self.target = target
        self.yaw_arr = yaw_arr
        self.yaw_data = yaw_data
        self.crosstrack_arr = crosstrack_arr
        self.crosstrack_data = crosstrack_data
        self.trail_line = trail_line
        self.trail_x = trail_x
        self.trail_y = trail_y

def animate(frame, fargs):

    ax = fargs.ax
    sim = fargs.sim
    path = fargs.path
    car = fargs.car
    desc = fargs.desc
    outline = fargs.outline
    fr = fargs.fr
    rr = fargs.rr
    fl = fargs.fl
    rl = fargs.rl
    rear_axle = fargs.rear_axle
    annotation = fargs.annotation
    target = fargs.target
    yaw_arr = fargs.yaw_arr
    yaw_data = fargs.yaw_data
    crosstrack_arr = fargs.crosstrack_arr
    crosstrack_data = fargs.crosstrack_data
    trail_line = fargs.trail_line
    trail_x = fargs.trail_x
    trail_y = fargs.trail_y

    ax[0].set_title(f'{sim.dt*frame:.2f}s', loc='right')

    # Static view: limits are set once in main()

    # Drive and draw car
    car.drive()
    outline_plot, fr_plot, rr_plot, fl_plot, rl_plot = desc.plot_car(car.x, car.y, car.yaw, car.delta)
    outline.set_data(outline_plot[0], outline_plot[1])
    fr.set_data(*fr_plot)
    rr.set_data(*rr_plot)
    fl.set_data(*fl_plot)
    rl.set_data(*rl_plot)
    rear_axle.set_data(car.x, car.y)

    # Update trail
    trail_x.append(car.x)
    trail_y.append(car.y)
    trail_line.set_data(trail_x, trail_y)

    # Show car's target
    target.set_data(path.px[car.target_id], path.py[car.target_id])

    # Annotate car's coordinate above car
    annotation.set_text(f"Crosstrack error: {car.crosstrack_error:.5f}")
    annotation.set_position((car.x - 10, car.y + 5))

    # Animate yaw
    yaw_arr.append(car.yaw)
    yaw_data.set_data(np.arange(frame + 1), yaw_arr)
    ax[1].set_ylim(yaw_arr[-1] - 5, yaw_arr[-1] + 5)

    # Animate crosstrack error
    crosstrack_arr.append(car.crosstrack_error)
    crosstrack_data.set_data(np.arange(frame + 1), crosstrack_arr)
    ax[2].set_ylim(crosstrack_arr[-1] - 1, crosstrack_arr[-1] + 1)

    return outline, fr, rr, fl, rl, rear_axle, target, yaw_data, crosstrack_data,

def main():
    
    sim = Simulation()
    path = Path()
    car = Car(path.px[0], path.py[0], path.pyaw[0], sim, path)
    desc = CarDescription(car.overall_length, car.overall_width, car.rear_overhang, car.tyre_diameter, car.tyre_width, car.axle_track, car.wheelbase)

    interval = sim.dt * 10**3

    # Fullscreen window
    fig = plt.figure()
    try:
        mgr = plt.get_current_fig_manager()
        mgr.full_screen_toggle()
    except Exception:
        pass

    # Layout using GridSpec: left column = map (spans both rows),
    # right column = yaw (top) and crosstrack (bottom)
    from matplotlib.gridspec import GridSpec
    gs = GridSpec(nrows=2, ncols=2, figure=fig, width_ratios=[1.0, 1.0], height_ratios=[1.0, 1.0])
    ax0 = fig.add_subplot(gs[:, 0])
    ax1 = fig.add_subplot(gs[0, 1])
    ax2 = fig.add_subplot(gs[1, 1])
    ax = [ax0, ax1, ax2]

    ax[0].set_aspect('equal')
    ax[0].plot(path.px, path.py, '--', color='gold')
    # Fit entire map in view with a margin
    try:
        margin = 20
        min_x, max_x = float(np.min(path.px)), float(np.max(path.px))
        min_y, max_y = float(np.min(path.py)), float(np.max(path.py))
        ax[0].set_xlim(min_x - margin, max_x + margin)
        ax[0].set_ylim(min_y - margin, max_y + margin)
    except Exception:
        pass

    annotation = ax[0].annotate(f"Crosstrack error: {float('inf')}", xy=(car.x - 10, car.y + 5), color='black', annotation_clip=False)
    target, = ax[0].plot([], [], '+r')

    outline, = ax[0].plot([], [], color='black')
    fr, = ax[0].plot([], [], color='black')
    rr, = ax[0].plot([], [], color='black')
    fl, = ax[0].plot([], [], color='black')
    rl, = ax[0].plot([], [], color='black')
    rear_axle, = ax[0].plot(car.x, car.y, '+', color='black', markersize=1)
    trail_line, = ax[0].plot([], [], color='blue', linewidth=1.5, alpha=0.8)
    trail_x, trail_y = [], []

    yaw_arr = []
    yaw_data, = ax[1].plot([], [])
    ax[1].set_xlim(0, sim.frames)
    ax[1].set_ylabel("Yaw")
    ax[1].grid()

    crosstrack_arr = []
    crosstrack_data, = ax[2].plot([], [])
    ax[2].set_xlim(0, sim.frames)
    ax[2].set_ylabel("Crosstrack error")
    ax[2].grid()

    fargs = [
        Fargs(
            ax=ax,
            sim=sim,
            path=path,
            car=car,
            desc=desc,
            outline=outline,
            fr=fr,
            rr=rr,
            fl=fl,
            rl=rl,
            rear_axle=rear_axle,
            annotation=annotation,
            target=target,
            yaw_arr=yaw_arr,
            yaw_data=yaw_data,
            crosstrack_arr=crosstrack_arr,
            crosstrack_data=crosstrack_data,
            trail_line=trail_line,
            trail_x=trail_x,
            trail_y=trail_y
        )
    ]

    anim = FuncAnimation(fig, animate, frames=sim.frames, init_func=lambda: None ,fargs=fargs, interval=interval, repeat=sim.loop)

    # Interactivity: pause/resume with space or 'p'; scroll to zoom on any axes
    paused = { 'value': False }

    def toggle_pause(*_):
        if paused['value']:
            anim.event_source.start()
        else:
            anim.event_source.stop()
        paused['value'] = not paused['value']

    def on_key(event):
        if event.key in (' ', 'p'):
            toggle_pause()

    def on_scroll(event):
        ax_current = event.inaxes
        if ax_current is None:
            return
        try:
            xdata = event.xdata
            ydata = event.ydata
            if xdata is None or ydata is None:
                return
            if event.button == 'up':
                scale = 1/1.2
            else:
                scale = 1.2
            xlim = ax_current.get_xlim()
            ylim = ax_current.get_ylim()
            new_width = (xlim[1]-xlim[0]) * scale
            new_height = (ylim[1]-ylim[0]) * scale
            relx = (xdata - xlim[0])/(xlim[1]-xlim[0])
            rely = (ydata - ylim[0])/(ylim[1]-ylim[0])
            ax_current.set_xlim([xdata - new_width*relx, xdata + new_width*(1-relx)])
            ax_current.set_ylim([ydata - new_height*rely, ydata + new_height*(1-rely)])
            ax_current.figure.canvas.draw_idle()
        except Exception:
            pass

    cid_key = fig.canvas.mpl_connect('key_press_event', on_key)
    cid_scroll = fig.canvas.mpl_connect('scroll_event', on_scroll)

    # Drag-to-pan on any axes
    pan_state = {
        'dragging': False,
        'ax': None,
        'press_x': None,
        'press_y': None,
        'orig_xlim': None,
        'orig_ylim': None,
    }

    def on_button_press(event):
        if event.button != 1:
            return
        if event.inaxes is None:
            return
        if event.xdata is None or event.ydata is None:
            return
        pan_state['dragging'] = True
        pan_state['ax'] = event.inaxes
        pan_state['press_x'] = event.xdata
        pan_state['press_y'] = event.ydata
        pan_state['orig_xlim'] = event.inaxes.get_xlim()
        pan_state['orig_ylim'] = event.inaxes.get_ylim()

    def on_button_release(event):
        if event.button != 1:
            return
        pan_state['dragging'] = False
        pan_state['ax'] = None

    def on_motion(event):
        if not pan_state['dragging']:
            return
        ax_current = pan_state['ax']
        if ax_current is None:
            return
        if event.inaxes != ax_current:
            return
        if event.xdata is None or event.ydata is None:
            return
        try:
            dx = event.xdata - pan_state['press_x']
            dy = event.ydata - pan_state['press_y']
            x0, x1 = pan_state['orig_xlim']
            y0, y1 = pan_state['orig_ylim']
            ax_current.set_xlim(x0 - dx, x1 - dx)
            ax_current.set_ylim(y0 - dy, y1 - dy)
            ax_current.figure.canvas.draw_idle()
        except Exception:
            pass

    cid_press = fig.canvas.mpl_connect('button_press_event', on_button_press)
    cid_release = fig.canvas.mpl_connect('button_release_event', on_button_release)
    cid_motion = fig.canvas.mpl_connect('motion_notify_event', on_motion)
    # anim.save('animation.gif', writer='imagemagick', fps=50)
    plt.show()

    print(f"Mean yaw: {np.mean(yaw_arr)}")
    print(f"Mean crosstrack error: {np.mean(crosstrack_arr)}")

if __name__ == '__main__':
    main()