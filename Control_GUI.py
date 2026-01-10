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

IDLE = ODrive_controller.IDLE
CLOSE_LOOP_CONTROL = ODrive_controller.CLOSED_LOOP_CONTROL

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ControlGUI")

POLL_INTERVAL_MS = 150  # GUI poll interval for status updates
PLOT_INTERVAL_MS = 50  # plot refresh interval

class ControlGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Control GUI")
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        # Display-only dummy controller (no hardware)
        self.controller = ODrive_controller.ODriveThread()
        self.controller.start()

        self.parm_labels = ["Control bandwidth:", "Encoder bandwidth:", "External load (kg):", "Load position (m):",
                  "Coulomb friction (Nm):", "Viscous friction (Nms/deg):", "Kp:", "Kd:"]
        self.param_vars = []

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
        self.fig = Figure(figsize=(6, 6), dpi=100) # 600x600 px là kích thước khởi tạo
        self.ax_pos = self.fig.add_subplot(311)
        self.ax_vel = self.fig.add_subplot(312)
        self.ax_tor = self.fig.add_subplot(313)
        self.fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Action buttons panel: top area for buttons ===================================================================================
        top_right = ttk.Frame(right, padding=6)
        top_right.pack(side=tk.TOP, fill=tk.X)

        for c in range(3): top_right.columnconfigure(c, weight=1)
        for r in range(2): top_right.rowconfigure(r, weight=1)

        # 1. Status Label
        self.status_lable = tk.Label(top_right, text="Ready", relief="ridge", bg="lightgreen")
        self.status_lable.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        # 2. Offset button
        self.btn_offset = tk.Button(top_right, text="Offset", bg="tomato", relief="raised", command=self._on_offset)
        self.btn_offset.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)
        # 3. IDLE/Close loop toggle
        self.btn_mode = tk.Button(top_right, text="Close Loop", bg="lightgreen", relief="raised")
        self.btn_mode.grid(row=1, column=0, sticky="nsew", padx=2, pady=2)
        # 4. Stop plotting: Chuyển từ ttk.Button sang tk.Button để đồng bộ hình dáng
        self.btn_plot = tk.Button(top_right, text="Stop plotting", relief="raised")
        self.btn_plot.grid(row=1, column=1, sticky="nsew", padx=2, pady=2)
        # 5. Reset 
        self.btn_reset = tk.Button(top_right, text="Reset", relief="raised")
        self.btn_reset.grid(row=2, column=0, sticky="nsew", padx=2, pady=2)    
        # 6. EStop
        self.btn_estop = tk.Button(top_right, text="ESTOP", bg="red", fg="white", relief="raised")
        self.btn_estop.grid(row=2, column=1, sticky="nsew", padx=2, pady=2)

        # Move controls ====================================================================================================================
        move_frame = ttk.LabelFrame(right, text="Move", padding=8)
        move_frame.pack(padx=6, pady=8, fill=tk.X)
        # Display position
        ttk.Label(move_frame, text="Position (deg):").grid(row=0, column=0, sticky=tk.W)
        self.entry_pos = ttk.Entry(move_frame, width=12, state="readonly")
        self.entry_pos.grid(row=0, column=1, padx=4, pady=2)
        # Enter target position
        ttk.Label(move_frame, text="Target (deg):").grid(row=1, column=0, sticky=tk.W)
        self.var_target = tk.StringVar() # Biến dùng để giữ giá trị
        self.entry_target = ttk.Entry(move_frame, textvariable=self.var_target, width=12, state="disabled")
        self.entry_target.grid(row=1, column=1, padx=4, pady=2)
        # Enter desired moving time
        ttk.Label(move_frame, text="Move time (s):").grid(row=2, column=0, sticky=tk.W)
        self.var_move_time = tk.StringVar(value="1.0") # Biến dùng để giữ giá trị
        self.entry_move_time = ttk.Entry(move_frame, textvariable=self.var_move_time, width=12, state="disabled")
        self.entry_move_time.grid(row=2, column=1, padx=4, pady=2)
        # Move button
        self.btn_move = ttk.Button(move_frame, text="Move")
        self.btn_move.grid(row=3, column=0, columnspan=2, pady=(6,0), sticky=tk.EW)

        # Error display =====================================================================================================================
        error_frame = ttk.LabelFrame(right, text="Error", padding=8)
        error_frame.pack(padx=6, pady=8, fill=tk.X)

        ttk.Label(error_frame, text="Position error (deg):").grid(row=0, column=0, sticky=tk.W)
        self.entry_pos_error = ttk.Entry(error_frame, width=12, state="readonly")
        self.entry_pos_error.grid(row=0, column=1, padx=4, pady=2)

        ttk.Label(error_frame, text="Velocity error (deg/s):").grid(row=1, column=0, sticky=tk.W)
        self.entry_vel_error = ttk.Entry(error_frame, width=12, state="readonly")
        self.entry_vel_error.grid(row=1, column=1, padx=4, pady=2)

        # Lower right: Send parameters block ================================================================================================
        param_frame = ttk.LabelFrame(right, text="Parameters", padding=8)
        param_frame.pack(padx=6, pady=8, fill=tk.X)

        # parameters: control bandwidth, encoder bandwidth, external load, load pos, coulomb friction, visc friction, Kp, Kd
        self._param_entries = []
        for i, lab in enumerate(self.parm_labels):
            ttk.Label(param_frame, text=lab).grid(row=i, column=0, sticky=tk.W, pady=1)
            v = tk.StringVar()
            ent = ttk.Entry(param_frame, textvariable=v, width=12, state="disabled")
            ent.grid(row=i, column=1, padx=4, pady=1)
            self.param_vars.append(v)
            self._param_entries.append(ent)

        self.btn_send_param = ttk.Button(param_frame, text="Send parameter")
        self.btn_send_param.grid(row=len(self.parm_labels), column=0, columnspan=2, pady=(6,0), sticky=tk.EW)

        # Status area at bottom
        status_frame = ttk.Frame(right, padding=6)
        status_frame.pack(side=tk.BOTTOM, fill=tk.X)

        self.status_text = tk.StringVar(value="Status: starting...")
        ttk.Label(status_frame, textvariable=self.status_text, relief=tk.RIDGE).pack(fill=tk.X)

        # Initialize some states
        self._update_param_entries(enabled=False)

        # Plot data lines
        self._pos_line, = self.ax_pos.plot([], [], label="q (deg)")
        self._pos_set_line, = self.ax_pos_set.plot([], [], label="q_d (deg)")
        self._vel_line, = self.ax_vel.plot([], [], label="qdot (deg/s)")
        self._vel_set_line, = self.ax_vel_set.plot([], [], label="qdot_d (deg/s)")
        self._tor_line, = self.ax_tor.plot([], [], label="Torque (Nm)")
        for ax in (self.ax_pos, self.ax_vel, self.ax_tor):
            ax.grid(True)
            ax.legend(loc="upper right")

        self._data_cache = []  # cached list of [ (t, pos, vel, tor), ... ]

    # -------------------------------------------------------------------------------------------------------------------------------------------
    # GUI callbacks
    # -------------------------------------------------------------------------------------------------------------------------------------------
    def _on_offset(self):
        self.controller.set_offset()
        # set color & enabled
        try:
            st = self.controller.is_offset
        except Exception:
            st = False
        if st: self.btn_offset.configure(state="disabled", bg="light green")
        else: self.btn_offset.configure(state="normal", bg="tomato")

    def _set_target_entry_enabled(self, enable):
        if enable:
            self.entry_target.configure(state="normal")
        else:
            self.entry_target.delete(0, tk.END)
            self.entry_target.configure(state="disabled")

    def _on_toggle_plot(self):
        if self.plotting:
            self.btn_plot.config(text="Stop plotting")
            self.plotting = False
        else:
            self.btn_plot.config(text="Continue plotting")
            self.plotting = True

    def _on_estop(self):
        if self.controller._estop_event.is_set():
            self.btn_estop.config(state="disabled")
        else: self.btn_estop.config(state="active")

        self.controller.emergency_stop()

    def _on_reset(self):
        self.controller.reset()

    def _on_move(self):
        target = self.var_target.get()
        moveTime = self.var_move_time.get()
        self.controller.moveTo(target, moveTime)

    def _on_send_parameters(self):
        # Disabled in display-only mode
        pass

    def _on_mode_tog(self):
        mode = self.controller.get_state()
        if mode == IDLE:
            self.btn_mode.config(text="Close Loop", bg="lightgreen", command=self.controller.enter_closed_loop)
            self._update_param_entries(True)
        else:
            self.btn_mode.config(text="IDLE", bg="yelow", command=self.controller.return_IDLE)
            self._update_param_entries(False)

    # ---------------------------
    # Background updates & plotting
    # ---------------------------
    

    
    def _poll_status(self):
        # Poll controller and update GUI widgets/buttons accordingly
        try:
            st = self.controller.get_state()
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

            # Enable/disable parameter entries depending on IDLE
            idle = st.get("idle", False)
            self._update_param_entries(enabled=idle)

            # Update ready/estop labels/colors
            if st.get("estop", False):
                self.status_lable.config(text="ESTOP", background="red")
            elif st.get("error", False):
                self.status_lable.config(text="ERROR", background="orange")
            elif st.get("connected", False):
                self.status_lable.config(text="Connected", background="lightgreen")
            else:
                self.status_lable.config(text="Disconnected", background="lightgrey")

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
        _state = "normal" if enabled else "disabled"
        for ent in getattr(self, "_param_entries", []):
            try:
                ent.configure(state=_state)
            except Exception:
                pass

    def _gather_plot_data(self):
        # read controller.get_data() and convert into arrays for plotting
        try:
            data = self.controller.get_data()
        except Exception:
            pass

        times, pos, vel, pos_set, vel_set, tor = zip(*data)
        t0 = times[0]
        times = [t-t0 for t in times]
        return times, pos, vel, pos_set, vel_set, tor

    def _plot_update(self):
        if self.plotting:
            times, pos, vel, pos_set, vel_set, tor = self._gather_plot_data()
            if times:
                # update lines
                self._pos_line.set_data(times, pos)
                self._pos_line.set_data(times, pos)
                self._vel_line.set_data(times, vel)
                self._vel_line.set_data(times, vel)
                self._tor_line.set_data(times, tor)

                # autoscale axes
                def autoscale(ax, x, y, margin=0.1):
                    if not x or not y:
                        return
                    ax.relim()
                    ax.autoscale_view()
                autoscale(self.ax_pos, times, pos)
                autoscale(self.ax_vel, times, vel)
                autoscale(self.ax_tor, times, tor)

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