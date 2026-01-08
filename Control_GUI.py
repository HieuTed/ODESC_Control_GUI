import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import logging
import math

# matplotlib embedding
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

# Dummy controller used for display-only GUI (no hardware access)
class DummyController:
    def __init__(self):
        self.is_offset = False
        self.closed_loop_control = False
        self._start_time = time.time()
        self._paused = False
    def start(self):
        # no-op for display-only
        pass
    def pause_logging(self):
        self._paused = True
    def resume_logging(self):
        self._paused = False
    def get_data(self):
        # generate sample time series (sine/cosine) for plotting
        t_now = time.time()
        xs = [t_now - i*0.05 for i in range(100)][::-1]
        data = []
        for tt in xs:
            dt = tt - self._start_time
            pos = 30.0 * math.sin(dt)
            vel = 30.0 * math.cos(dt)
            tor = 0.5 * math.sin(dt*0.5)
            data.append((tt, pos, vel, tor))
        return data
    def get_status(self):
        # return a static-ish status for display
        return {"pos": 0.0, "idle": True, "estop": False, "error": False, "connected": True, "closed_loop": False, "is_offset": self.is_offset}
    def shutdown(self, wait_timeout=0):
        pass
    def is_idle(self):
        return True
    def set_offset(self):
        self.is_offset = True
        return True
    def apply_move(self, *args, **kwargs):
        return True
    def send_parameters(self, *args, **kwargs):
        return True
    def emergency_stop(self):
        pass
    def reset_estop(self):
        pass
    def clear_error(self):
        pass

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ControlGUI")

POLL_INTERVAL_MS = 150  # GUI poll interval for status updates
PLOT_INTERVAL_MS = 200  # plot refresh interval

class ControlGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Control GUI")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Display-only dummy controller (no hardware)
        self.controller = DummyController()
        self.controller.start()

        # UI state
        self.plotting = True
        self._last_plot_time = 0.0

        # Build UI
        self._build_ui()

        # Start periodic polls
        self.after(POLL_INTERVAL_MS, self._poll_status)
        self.after(PLOT_INTERVAL_MS, self._plot_update)

    def _build_ui(self):
        # Main frame layout: left plots, right controls
        main = ttk.Frame(self, padding=6)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = ttk.Frame(main, width=360)
        right.pack(side=tk.RIGHT, fill=tk.Y)

        # Plots: 3 stacked subplots (Position, Velocity, Torque)
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax_pos = self.fig.add_subplot(311)
        self.ax_vel = self.fig.add_subplot(312)
        self.ax_tor = self.fig.add_subplot(313)
        self.fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Right panel: top area for buttons
        top_right = ttk.Frame(right, padding=6)
        top_right.pack(side=tk.TOP, fill=tk.X)

        # Offset button (red when not offset, green and disabled when offset)
        self.btn_offset = tk.Button(top_right, text="Offset", width=10)
        self.btn_offset.configure(bg="tomato")  # red-ish initially
        self.btn_offset.pack(pady=(2, 6))

        # Stop plotting / Continue toggle
        self.btn_plot = ttk.Button(top_right, text="Stop plotting")
        self.btn_plot.pack(fill=tk.X, pady=(0,6))

        # Close loop toggle
        self.btn_cloop = tk.Button(top_right, text="Close Loop", width=10, bg="lightgreen")
        self.btn_cloop.pack(pady=(0,6))

        # ESTOP / Reset / Ready area
        estop_frame = ttk.Frame(top_right)
        estop_frame.pack(pady=(6,2), fill=tk.X)
        self.btn_estop = tk.Button(estop_frame, text="ESTOP", bg="red", fg="white", width=8)
        self.btn_estop.pack(side=tk.LEFT, padx=(0,4))
        self.btn_reset = tk.Button(estop_frame, text="Reset", width=8)
        self.btn_reset.pack(side=tk.LEFT)
        self.label_ready = ttk.Label(top_right, text="Ready", relief=tk.SUNKEN, width=12)
        self.label_ready.pack(pady=(6,2))

        # Middle right: Move controls (Position display, Target entry, Move time, Move button)
        move_frame = ttk.LabelFrame(right, text="Move", padding=8)
        move_frame.pack(padx=6, pady=8, fill=tk.X)

        ttk.Label(move_frame, text="Position (deg):").grid(row=0, column=0, sticky=tk.W)
        self.entry_pos = ttk.Entry(move_frame, width=12, state="readonly")
        self.entry_pos.grid(row=0, column=1, padx=4, pady=2)

        ttk.Label(move_frame, text="Target (deg):").grid(row=1, column=0, sticky=tk.W)
        self.var_target = tk.StringVar()
        self.entry_target = ttk.Entry(move_frame, textvariable=self.var_target, width=12, state="disabled")
        self.entry_target.grid(row=1, column=1, padx=4, pady=2)

        ttk.Label(move_frame, text="Move time (s):").grid(row=2, column=0, sticky=tk.W)
        self.var_move_time = tk.StringVar(value="1.0")
        self.entry_move_time = ttk.Entry(move_frame, textvariable=self.var_move_time, width=12, state="disabled")
        self.entry_move_time.grid(row=2, column=1, padx=4, pady=2)

        self.btn_move = ttk.Button(move_frame, text="Move")
        self.btn_move.grid(row=3, column=0, columnspan=2, pady=(6,0), sticky=tk.EW)

        # Lower right: Send parameters block
        param_frame = ttk.LabelFrame(right, text="Parameters", padding=8)
        param_frame.pack(padx=6, pady=8, fill=tk.X)

        # parameters: control bandwidth, encoder bandwidth, external load, load pos, coulomb friction, visc friction, Kp, Kd
        labels = ["Control bandwidth:", "Encoder bandwidth:", "External load (kg):", "Load position (m):",
                  "Coulomb friction (Nm):", "Viscous friction (Nms/deg):", "Kp:", "Kd:"]
        self.param_vars = []
        self._param_entries = []
        for i, lab in enumerate(labels):
            ttk.Label(param_frame, text=lab).grid(row=i, column=0, sticky=tk.W, pady=1)
            v = tk.StringVar()
            ent = ttk.Entry(param_frame, textvariable=v, width=12, state="disabled")
            ent.grid(row=i, column=1, padx=4, pady=1)
            self.param_vars.append(v)
            self._param_entries.append(ent)

        self.btn_send_param = ttk.Button(param_frame, text="Send parameter")
        self.btn_send_param.grid(row=len(labels), column=0, columnspan=2, pady=(6,0), sticky=tk.EW)

        # Status area at bottom
        status_frame = ttk.Frame(right, padding=6)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X)

        self.status_text = tk.StringVar(value="Status: starting...")
        ttk.Label(status_frame, textvariable=self.status_text, relief=tk.RIDGE).pack(fill=tk.X)

        # Initialize some states
        self._update_offset_button()
        self._update_param_entries(enabled=False)

        # Plot data lines
        self._pos_line, = self.ax_pos.plot([], [], label="Position (deg)")
        self._vel_line, = self.ax_vel.plot([], [], label="Velocity (deg/s)")
        self._tor_line, = self.ax_tor.plot([], [], label="Torque (Nm)")
        for ax in (self.ax_pos, self.ax_vel, self.ax_tor):
            ax.grid(True)

        self._data_cache = []  # cached list of [ (t, pos, vel, tor), ... ]

    # ---------------------------
    # GUI callbacks
    # ---------------------------
    def _on_offset(self):
        # Disabled in display-only mode
        pass

    def _update_offset_button(self):
        # set color & enabled based on controller.is_offset
        try:
            st = self.controller.is_offset
        except Exception:
            st = False
        if st:
            self.btn_offset.configure(state="disabled", bg="light green")
            self._set_target_entry_enabled(True)
        else:
            self.btn_offset.configure(state="normal", bg="tomato")
            self._set_target_entry_enabled(False)

    def _set_target_entry_enabled(self, enable):
        if enable:
            self.entry_target.configure(state="normal")
        else:
            self.entry_target.delete(0, tk.END)
            self.entry_target.configure(state="disabled")

    def _on_toggle_plot(self):
        # Disabled in display-only mode
        pass

    def _on_toggle_cloop(self):
        # Disabled in display-only mode
        pass

    def _on_estop(self):
        # Disabled in display-only mode
        pass

    def _on_reset(self):
        # Disabled in display-only mode
        pass

    def _on_move(self):
        # Disabled in display-only mode
        pass

    def _on_send_parameters(self):
        # Disabled in display-only mode
        pass

    # ---------------------------
    # Background updates & plotting
    # ---------------------------
    def _poll_status(self):
        # Poll controller and update GUI widgets/buttons accordingly
        try:
            st = self.controller.get_status()
        except Exception:
            st = None

        if st is None:
            self.status_text.set("Status: controller unavailable")
        else:
            # Update position display (read-only)
            pos = st.get("pos", 0.0)
            self.entry_pos.config(state="normal")
            self.entry_pos.delete(0, tk.END)
            self.entry_pos.insert(0, f"{pos:.3f}")
            self.entry_pos.config(state="readonly")

            # Offset button update
            self._update_offset_button()

            # Enable/disable parameter entries depending on IDLE
            idle = st.get("idle", False)
            self._update_param_entries(enabled=idle)

            # Update ready/estop labels/colors
            if st.get("estop", False):
                self.label_ready.config(text="ESTOP", background="red")
            elif st.get("error", False):
                self.label_ready.config(text="ERROR", background="orange")
            elif st.get("connected", False):
                self.label_ready.config(text="Connected", background="lightgreen")
            else:
                self.label_ready.config(text="Disconnected", background="lightgrey")

            # Update Close Loop button text to indicate current state
            if st.get("closed_loop", False):
                # per requirement, display "IDLE" when in closed-loop
                self.btn_cloop.configure(text="IDLE")
            else:
                self.btn_cloop.configure(text="Close Loop")

            # If offset set, ensure target entry enabled
            if st.get("is_offset", False):
                self.entry_target.configure(state="normal")
            else:
                self.entry_target.configure(state="disabled")

            # update status text if not overwritten recently
            if "Status:" not in self.status_text.get():
                # keep existing status unless action updated it
                pass

        # schedule next poll
        self.after(POLL_INTERVAL_MS, self._poll_status)

    def _update_param_entries(self, enabled):
        state = "normal" if enabled else "disabled"
        for ent in getattr(self, "_param_entries", []):
            try:
                ent.configure(state=state)
            except Exception:
                pass
        # Also ensure Move-related entries follow offset rule:
        if self.controller.is_offset:
            self.entry_target.configure(state="normal")
        else:
            self.entry_target.configure(state="disabled")

    def _gather_plot_data(self):
        # read controller.get_data() and convert into arrays for plotting
        try:
            data = self.controller.get_data()
        except Exception:
            data = []
        if not data:
            return [], [], [], []
        times = [d[0] for d in data]
        t0 = times[0]
        xs = [t - t0 for t in times]
        pos = [d[1] for d in data]
        vel = [d[2] for d in data]
        tor = [d[3] for d in data]
        return xs, pos, vel, tor

    def _plot_update(self):
        if self.plotting:
            xs, pos, vel, tor = self._gather_plot_data()
            if xs:
                # update lines
                self._pos_line.set_data(xs, pos)
                self._vel_line.set_data(xs, vel)
                self._tor_line.set_data(xs, tor)

                # autoscale axes
                def autoscale(ax, x, y, margin=0.1):
                    if not x or not y:
                        return
                    ax.relim()
                    ax.autoscale_view()
                autoscale(self.ax_pos, xs, pos)
                autoscale(self.ax_vel, xs, vel)
                autoscale(self.ax_tor, xs, tor)

                self.ax_pos.set_ylabel("deg")
                self.ax_vel.set_ylabel("deg/s")
                self.ax_tor.set_ylabel("Nm")
                self.ax_tor.set_xlabel("Time (s)")
                self.canvas.draw_idle()

        # schedule next update
        self.after(PLOT_INTERVAL_MS, self._plot_update)

    # ---------------------------
    # On-close / cleanup
    # ---------------------------
    def _on_close(self):
        # Confirm close
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            try:
                # Ensure torque -> 0 and IDLE, then shutdown controller
                self.controller.shutdown(wait_timeout=2.0)
            except Exception:
                logger.exception("Error shutting down controller")
            self.destroy()

if __name__ == "__main__":
    app = ControlGUI()
    app.mainloop()