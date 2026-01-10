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
import ODrive_controller

# Map constants from controller module (kept for readability)
IDLE = ODrive_controller.IDLE
CLOSE_LOOP_CONTROL = ODrive_controller.CLOSED_LOOP_CONTROL

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ControlGUI")

# Combined update interval (GUI + plot)
UPDATE_INTERVAL_MS = 50

class ControlGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Control GUI")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Display-only dummy controller (or real, depending on underlying module)
        self.controller = ODrive_controller.ODriveThread()
        self.controller.start()

        self.parm_labels = ["Control bandwidth:", "Encoder bandwidth:", "External load (kg):", "Load position (m):",
                  "Coulomb friction (Nm):", "Viscous friction (Nms/deg):", "Kp:", "Kd:"]
        self.param_vars = []

        # UI state
        self.plotting = True

        # Build UI
        self._build_ui()

        # Start periodic combined updates
        self.after(UPDATE_INTERVAL_MS, self._update)

    def _build_ui(self):
        # Main frame layout: left plots, right controls
        main = ttk.Frame(self, padding=6)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        right = ttk.Frame(main, width=360)
        right.pack(side=tk.RIGHT, fill=tk.Y)

        # Plots: 3 stacked subplots (Position, Velocity, Torque)
        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax_pos = self.fig.add_subplot(311)
        self.ax_vel = self.fig.add_subplot(312)
        self.ax_tor = self.fig.add_subplot(313)
        self.fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Action buttons panel
        top_right = ttk.Frame(right, padding=6)
        top_right.pack(side=tk.TOP, fill=tk.X)

        for c in range(3): top_right.columnconfigure(c, weight=1)
        for r in range(3): top_right.rowconfigure(r, weight=1)

        # 1. Status Label
        self.status_label = tk.Label(top_right, text="Ready", relief="ridge", bg="lightgreen")
        self.status_label.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)

        # 2. Offset button
        self.btn_offset = tk.Button(top_right, text="Offset", bg="tomato", relief="raised", command=self._on_offset)
        self.btn_offset.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)

        # 3. IDLE/Close loop toggle (mode)
        self.btn_mode = tk.Button(top_right, text="Close Loop", bg="lightgreen", relief="raised", command=self._on_mode_tog)
        self.btn_mode.grid(row=1, column=0, sticky="nsew", padx=2, pady=2)

        # 4. Stop/Continue plotting
        self.btn_plot = tk.Button(top_right, text="Stop plotting", relief="raised", command=self._on_toggle_plot)
        self.btn_plot.grid(row=1, column=1, sticky="nsew", padx=2, pady=2)

        # 5. Reset
        self.btn_reset = tk.Button(top_right, text="Reset", relief="raised", command=self._on_reset)
        self.btn_reset.grid(row=2, column=0, sticky="nsew", padx=2, pady=2)

        # 6. EStop
        self.btn_estop = tk.Button(top_right, text="ESTOP", bg="red", fg="white", relief="raised", command=self._on_estop)
        self.btn_estop.grid(row=2, column=1, sticky="nsew", padx=2, pady=2)

        # Move controls
        move_frame = ttk.LabelFrame(right, text="Move", padding=8)
        move_frame.pack(padx=6, pady=8, fill=tk.X)
        # Display position
        ttk.Label(move_frame, text="Position (deg):").grid(row=0, column=0, sticky=tk.W)
        self.entry_pos = ttk.Entry(move_frame, width=12, state="readonly")
        self.entry_pos.grid(row=0, column=1, padx=4, pady=2)
        # Enter target position
        ttk.Label(move_frame, text="Target (deg):").grid(row=1, column=0, sticky=tk.W)
        self.var_target = tk.StringVar()
        self.entry_target = ttk.Entry(move_frame, textvariable=self.var_target, width=12, state="disabled")
        self.entry_target.grid(row=1, column=1, padx=4, pady=2)
        # Enter desired moving time
        ttk.Label(move_frame, text="Move time (s):").grid(row=2, column=0, sticky=tk.W)
        self.var_move_time = tk.StringVar(value="5.0")
        self.entry_move_time = ttk.Entry(move_frame, textvariable=self.var_move_time, width=12, state="disabled")
        self.entry_move_time.grid(row=2, column=1, padx=4, pady=2)
        # Move button
        self.btn_move = ttk.Button(move_frame, text="Move", command=self._on_move)
        self.btn_move.grid(row=3, column=0, columnspan=2, pady=(6,0), sticky=tk.EW)

        # Error display
        error_frame = ttk.LabelFrame(right, text="Error", padding=8)
        error_frame.pack(padx=6, pady=8, fill=tk.X)

        ttk.Label(error_frame, text="Position error (deg):").grid(row=0, column=0, sticky=tk.W)
        self.entry_pos_error = ttk.Entry(error_frame, width=12, state="readonly")
        self.entry_pos_error.grid(row=0, column=1, padx=4, pady=2)

        ttk.Label(error_frame, text="Velocity error (deg/s):").grid(row=1, column=0, sticky=tk.W)
        self.entry_vel_error = ttk.Entry(error_frame, width=12, state="readonly")
        self.entry_vel_error.grid(row=1, column=1, padx=4, pady=2)

        # Send parameters block
        param_frame = ttk.LabelFrame(right, text="Parameters", padding=8)
        param_frame.pack(padx=6, pady=8, fill=tk.X)

        self._param_entries = []
        for i, lab in enumerate(self.parm_labels):
            ttk.Label(param_frame, text=lab).grid(row=i, column=0, sticky=tk.W, pady=1)
            v = tk.StringVar()
            ent = ttk.Entry(param_frame, textvariable=v, width=12, state="disabled")
            ent.grid(row=i, column=1, padx=4, pady=1)
            self.param_vars.append(v)
            self._param_entries.append(ent)

        self.btn_send_param = ttk.Button(param_frame, text="Send parameter", command=self._on_send_parameters)
        self.btn_send_param.grid(row=len(self.parm_labels), column=0, columnspan=2, pady=(6,0), sticky=tk.EW)

        # Status area at bottom
        status_frame = ttk.Frame(right, padding=6)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X)

        self.status_text = tk.StringVar(value="Status: starting...")
        ttk.Label(status_frame, textvariable=self.status_text, relief=tk.RIDGE).pack(fill=tk.X)

        # Initialize some states
        self._update_param_entries(enabled=False)

        # Plot data lines (empty initially)
        self._pos_line, = self.ax_pos.plot([], [], label="q (deg)")
        self._pos_set_line, = self.ax_pos.plot([], [], label="q_d (deg)", linestyle="--")
        self._vel_line, = self.ax_vel.plot([], [], label="qdot (deg/s)")
        self._vel_set_line, = self.ax_vel.plot([], [], label="qdot_d (deg/s)", linestyle="--")
        self._tor_line, = self.ax_tor.plot([], [], label="Torque (Nm)")
        for ax in (self.ax_pos, self.ax_vel, self.ax_tor):
            ax.grid(True)
            ax.legend(loc="upper right")

        # Keep a reference to last plotted time origin to keep plots stable
        self._last_t0 = None

    # ---------------------------------------------------------------------------
    # GUI callbacks
    # ---------------------------------------------------------------------------
    def _on_offset(self):
        try:
            self.controller.set_offset()
            # set color & enabled
            if getattr(self.controller, "isOffset", False):
                self.btn_offset.configure(state="disabled", bg="lightgreen")
                self.entry_target.configure(state="normal")
            else:
                self.btn_offset.configure(state="normal", bg="tomato")
        except Exception:
            logger.exception("Offset error")

    def _set_target_entry_enabled(self, enable):
        if enable:
            self.entry_target.configure(state="normal")
        else:
            self.entry_target.delete(0, tk.END)
            self.entry_target.configure(state="disabled")

    def _on_toggle_plot(self):
        self.plotting = not self.plotting
        if self.plotting:
            self.btn_plot.config(text="Stop plotting")
        else:
            self.btn_plot.config(text="Continue plotting")

    def _on_estop(self):
        try:
            self.controller.emergency_stop()
            self.btn_estop.config(state="disabled")
        except Exception:
            logger.exception("EStop error")

    def _on_reset(self):
        try:
            self.controller.reset()
            # re-enable estop button after reset
            self.btn_estop.config(state="normal")
        except Exception:
            logger.exception("Reset error")

    def _on_move(self):
        try:
            target = float(self.var_target.get())
            moveTime = float(self.var_move_time.get())
        except Exception:
            messagebox.showerror("Input error", "Target and Move time must be numeric")
            return
        try:
            self.controller.moveTo(target, moveTime)
        except Exception:
            logger.exception("Move error")

    def _on_send_parameters(self):
        # Read parameter entries and send to controller.update_parameter
        try:
            vals = []
            for v in self.param_vars:
                s = v.get().strip()
                if s == "":
                    vals.append(0.0)
                else:
                    # treat as float except first two (bandwidths) -> int
                    vals.append(float(s))
            if len(vals) < 8:
                messagebox.showwarning("Parameters", "Please fill all parameter fields")
                return
            # Convert first two to int (bandwidths)
            params = [int(vals[0]), int(vals[1])] + vals[2:8]
            self.controller.update_parameter(*params)
            self.status_text.set("Status: parameters sent")
        except Exception:
            logger.exception("Send parameters error")
            messagebox.showerror("Error", "Failed to send parameters to controller")

    def _on_mode_tog(self):
        # Toggle closed loop / IDLE on controller
        try:
            if getattr(self.controller, "closed_loop_control", False):
                # currently closed -> go to IDLE
                self.controller.return_IDLE()
                self.btn_mode.config(text="Close Loop", bg="lightgreen")
                self._update_param_entries(False)
            else:
                # try to enter closed loop
                self.controller.enter_closed_loop()
                self.btn_mode.config(text="IDLE", bg="yellow")
                self._update_param_entries(True)
        except Exception:
            logger.exception("Mode toggle error")

    # ---------------------------------------------------------------------------
    # Combined update: poll controller state, update GUI widgets, update plots
    # ---------------------------------------------------------------------------
    def _update(self):
        try:
            # Build a state dict from controller attributes (safe access)
            try:
                connected = bool(getattr(self.controller, "connected", False))
                closed_loop = bool(getattr(self.controller, "closed_loop_control", False))
                is_offset = bool(getattr(self.controller, "isOffset", False))
                estop = bool(getattr(self.controller, "_estop_event", threading.Event()).is_set())
                error = bool(getattr(self.controller, "error", False))
            except Exception:
                connected = closed_loop = is_offset = estop = error = False

            # Update status label (color + text)
            if estop:
                self.status_label.config(text="ESTOP", background="red")
            elif error:
                self.status_label.config(text="ERROR", background="orange")
            elif connected:
                self.status_label.config(text="Connected", background="lightgreen")
            else:
                self.status_label.config(text="Disconnected", background="lightgrey")

            # Update mode button text
            if closed_loop:
                self.btn_mode.config(text="IDLE", bg="yellow")
            else:
                self.btn_mode.config(text="Close Loop", bg="lightgreen")

            # Enable/disable parameter entries depending on IDLE (idle == not closed-loop)
            self._update_param_entries(enabled=(not closed_loop))

            # Enable/disable target entry depending on offset
            if is_offset:
                self._set_target_entry_enabled(True)
                self.btn_offset.configure(state="disabled", bg="lightgreen")
            else:
                self._set_target_entry_enabled(False)
                self.btn_offset.configure(state="normal", bg="tomato")

            # Get data for plotting and display
            data = []
            try:
                data = self.controller.get_data()
            except Exception:
                data = []

            if data:
                # data is list of tuples: (time, pos, vel, pos_set, vel_set, tor)
                times, pos_vals, vel_vals, pos_set_vals, vel_set_vals, tor_vals = zip(*data)
                # normalize time origin to the first sample in the list (keeps plotting stable)
                t0 = times[0] if self._last_t0 is None else self._last_t0
                # If a big discontinuity, reset origin to current first sample
                if self._last_t0 is None or (times[-1] - t0) > 30.0:
                    t0 = times[0]
                    self._last_t0 = t0
                times_rel = [t - t0 for t in times]

                # Update plot lines if plotting enabled
                if self.plotting:
                    self._pos_line.set_data(times_rel, pos_vals)
                    self._pos_set_line.set_data(times_rel, pos_set_vals)
                    self._vel_line.set_data(times_rel, vel_vals)
                    self._vel_set_line.set_data(times_rel, vel_set_vals)
                    self._tor_line.set_data(times_rel, tor_vals)

                    # autoscale each axis
                    def autoscale(ax, x, y):
                        if not x or not y:
                            return
                        ax.relim()
                        ax.autoscale_view()
                    autoscale(self.ax_pos, times_rel, pos_vals + pos_set_vals if isinstance(pos_vals, tuple) else pos_vals)
                    autoscale(self.ax_vel, times_rel, vel_vals + vel_set_vals if isinstance(vel_vals, tuple) else vel_vals)
                    autoscale(self.ax_tor, times_rel, tor_vals)

                    self.ax_pos.set_ylabel("deg")
                    self.ax_vel.set_ylabel("deg/s")
                    self.ax_tor.set_ylabel("Nm")
                    self.ax_tor.set_xlabel("Time (s)")
                    self.canvas.draw_idle()

                # Update displayed numeric values from the latest sample
                last_t, last_pos, last_vel, last_pos_set, last_vel_set, last_tor = data[-1]
                # Show current position
                try:
                    self.entry_pos.config(state="normal")
                    self.entry_pos.delete(0, tk.END)
                    self.entry_pos.insert(0, f"{last_pos:.3f}")
                    self.entry_pos.config(state="readonly")
                except Exception:
                    pass

                # Update errors (setpoint minus actual)
                pos_err = last_pos_set - last_pos
                vel_err = last_vel_set - last_vel
                try:
                    self.entry_pos_error.config(state="normal")
                    self.entry_pos_error.delete(0, tk.END)
                    self.entry_pos_error.insert(0, f"{pos_err:.3f}")
                    self.entry_pos_error.config(state="readonly")

                    self.entry_vel_error.config(state="normal")
                    self.entry_vel_error.delete(0, tk.END)
                    self.entry_vel_error.insert(0, f"{vel_err:.3f}")
                    self.entry_vel_error.config(state="readonly")
                except Exception:
                    pass

            else:
                # No data: clear plots (or leave previous)
                pass

            # Update status text minimally
            if connected:
                self.status_text.set("Status: running")
            else:
                self.status_text.set("Status: controller unavailable")

        except Exception:
            logger.exception("Update loop error")

        # schedule next update
        self.after(UPDATE_INTERVAL_MS, self._update)

    def _update_param_entries(self, enabled):
        _state = "normal" if enabled else "disabled"
        for ent in getattr(self, "_param_entries", []):
            try:
                ent.configure(state=_state)
            except Exception:
                pass

    # ---------------------------
    # On-close / cleanup
    # ---------------------------
    def _on_close(self):
        # Confirm close
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            try:
                # Attempt graceful shutdown of controller thread
                try:
                    # prefer stop() which exists in the provided controller
                    self.controller.stop()
                    # give it a moment and join
                    if hasattr(self.controller, "join"):
                        self.controller.join(timeout=2.0)
                except Exception:
                    logger.exception("Error shutting down controller")
            finally:
                self.destroy()

if __name__ == "__main__":
    app = ControlGUI()
    app.mainloop()