# -*- coding: utf-8 -*-
"""
ODrive_controller.py
Controller module with trajectory planning (trapezoidal) for a 1-DOF ODrive-driven mechanism.

Key additions:
- Trajectory base class and TrapezoidalTrajectory implementation.
- apply_move() now starts a trajectory; run() will follow setpoints from trajectory.
- Handles vmax limit by adjusting trapezoid parameters, falling back to triangular
  or extending required duration if necessary.
- Units: degrees for position, deg/s for velocity, deg/s^2 for acceleration.
"""

from collections import deque
import threading
import time
import math
import logging

# Optional odrive import (guarded)
try:
    import odrive
    from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE
except Exception:
    odrive = None
    AXIS_STATE_CLOSED_LOOP_CONTROL = 3
    AXIS_STATE_IDLE = 1

logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())

G = 9.81

# -----------------------------
# Trajectory abstractions
# -----------------------------
class Trajectory:
    """
    Base class / interface for trajectories.
    Implementations must provide:
      - duration (seconds, float)
      - sample(t) -> (pos_deg, vel_deg_s, acc_deg_s2) for 0 <= t <= duration
    and optional:
      - start() / stop() hooks or stateful behavior.
    """
    def __init__(self):
        self.duration = 0.0

    def sample(self, t):
        raise NotImplementedError("Trajectory.sample must be implemented")

class TrapezoidalTrajectory(Trajectory):
    """
    Symmetric trapezoidal trajectory (accelerate -> cruise -> decelerate)
    with acceleration/deceleration times ta each, cruise time tc, total T.

    Parameters:
      start_pos (deg), end_pos (deg), duration (s),
      accel_fraction (fraction of total time for accel; default 0.3),
      vmax_limit (deg/s) maximum allowed cruising velocity.

    Behavior:
      - Prefer to use accel_fraction * duration as ta.
      - Compute vmax needed to cover distance in the given duration.
      - If vmax needed > vmax_limit, attempt to use vmax_limit and adjust ta:
         * compute ta' = T - D/vmax_limit. If 0 <= ta' <= T/2, use trapezoid with ta'.
         * else try triangular profile (tc=0, ta=T/2) with required peak vp = 2D/T; if vp <= vmax_limit use it.
         * else it's impossible to complete in T with vmax_limit: extend duration to T_min = 2D/vmax_limit and build triangular reaching vmax_limit.
    Note:
      - No explicit acceleration limit; acceleration a = vmax / ta may be large if ta small.
      - Sign handling included for motion direction.
    """
    def __init__(self, start_pos, end_pos, duration, accel_fraction=0.3, vmax_limit=None):
        super().__init__()
        self.start = float(start_pos)
        self.end = float(end_pos)
        self.duration = float(duration) if duration is not None and duration > 0 else 0.0
        self.accel_fraction = float(accel_fraction)
        self.vmax_limit = float(vmax_limit) if vmax_limit is not None else None

        # computed parameters
        self.dir_sign = 1.0 if (self.end - self.start) >= 0 else -1.0
        self.D = abs(self.end - self.start)  # distance in degrees (non-negative)

        # Handle zero-distance immediate return
        if self.D == 0.0 or self.duration == 0.0:
            # zero-motion profile
            self.ta = 0.0
            self.tc = 0.0
            self.vmax = 0.0
            self.duration = 0.0
            return

        # initial nominal ta based on fraction
        ta_nom = self.accel_fraction * self.duration
        # ensure ta_nom <= T/2
        ta_nom = min(ta_nom, self.duration / 2.0)

        # Try naively compute vmax for trapezoid with nominal ta
        # Derived formula: D = vmax * (T - ta)  => vmax = D / (T - ta)
        denom = max(self.duration - ta_nom, 1e-12)
        vmax_trap = self.D / denom

        # Decide final parameters
        if self.vmax_limit is None or vmax_trap <= self.vmax_limit:
            # feasible without exceeding limit; use nominal ta
            self.ta = ta_nom
            self.tc = self.duration - 2.0 * self.ta
            self.vmax = vmax_trap
            # ensure non-negative tc
            if self.tc < 0:
                # fallback to triangular with ta = T/2
                self.ta = self.duration / 2.0
                self.tc = 0.0
                self.vmax = 2.0 * self.D / self.duration
        else:
            # vmax_trap exceeds limit -> attempt to use vmax_limit and adjust ta
            vmax_lim = self.vmax_limit
            # compute candidate ta' = T - D / vmax_limit
            ta_candidate = self.duration - (self.D / vmax_lim)
            # ta_candidate must be between 0 and T/2
            if 0.0 <= ta_candidate <= (self.duration / 2.0):
                self.ta = ta_candidate
                self.tc = self.duration - 2.0 * self.ta
                self.vmax = vmax_lim
            else:
                # try triangular with ta = T/2 -> peak vp = 2D/T
                vp_req = 2.0 * self.D / self.duration
                if vp_req <= vmax_lim:
                    # triangular feasible within vmax limit
                    self.ta = self.duration / 2.0
                    self.tc = 0.0
                    self.vmax = vp_req
                else:
                    # Not achievable within requested duration while respecting vmax_limit.
                    # Compute minimal duration to achieve motion with vmax_limit using triangular profile:
                    # For triangular with peak vp = vmax_limit and ta = vp / a (but here we free accel),
                    # minimal duration T_min such that vp = vmax_limit and D = vp * T_min / 2 => T_min = 2D/vmax_limit
                    T_min = 2.0 * self.D / vmax_lim
                    logger.warning("Requested duration too short for vmax_limit: extending duration from %.3f to %.3f",
                                   self.duration, T_min)
                    # rebuild as triangular with duration = T_min
                    self.duration = T_min
                    self.ta = self.duration / 2.0
                    self.tc = 0.0
                    self.vmax = vmax_lim

        # final safety clamps
        self.ta = max(self.ta, 0.0)
        self.tc = max(self.tc, 0.0)
        self.vmax = max(self.vmax, 0.0)
        # acceleration (deg/s^2)
        self.a = (self.vmax / self.ta) if self.ta > 1e-12 else float('inf')  # may be inf for instantaneous step

    def sample(self, t):
        """
        Sample trajectory at elapsed time t (seconds from start).
        Returns (pos_deg, vel_deg_s, acc_deg_s2).
        Handles t < 0 and t > duration (clamps to endpoints).
        """
        if self.D == 0.0 or self.duration == 0.0:
            # trivial stationary
            return (self.start, 0.0, 0.0)

        # clamp t
        if t <= 0.0:
            return (self.start, 0.0, 0.0)
        if t >= self.duration:
            return (self.end, 0.0, 0.0)

        ta = self.ta
        tc = self.tc
        T = self.duration
        vmax = self.vmax
        sign = self.dir_sign

        # times:
        t_acc_end = ta
        t_cruise_end = ta + tc
        # acceleration a:
        a = (vmax / ta) if ta > 1e-12 else 0.0

        # compute position along positive direction; then multiply by sign and add start
        if t < t_acc_end:
            # accelerating
            vel = a * t
            pos_rel = 0.5 * a * (t ** 2)
            acc = a
        elif t < t_cruise_end:
            # cruising
            vel = vmax
            pos_acc = 0.5 * a * (ta ** 2)  # distance during accel
            pos_rel = pos_acc + vmax * (t - ta)
            acc = 0.0
        else:
            # decelerating
            td = t - t_cruise_end
            # decel time remaining until end: t_rem = T - t
            # velocity = vmax - a*td
            vel = max(vmax - a * td, 0.0)
            # distance decel: distance during decel portion upto td: vmax*td - 0.5*a*td^2
            pos_acc = 0.5 * a * (ta ** 2)
            pos_cruise = vmax * tc
            pos_rel = pos_acc + pos_cruise + (vmax * td - 0.5 * a * (td ** 2))
            acc = -a

        # apply sign and offset to get absolute position
        pos = self.start + sign * pos_rel
        vel_signed = sign * vel
        acc_signed = sign * acc
        return (pos, vel_signed, acc_signed)

    def __repr__(self):
        return ("<TrapezoidalTrajectory start={:.3f} end={:.3f} T={:.3f} ta={:.3f} tc={:.3f} vmax={:.3f}>"
                .format(self.start, self.end, self.duration, self.ta, self.tc, self.vmax))

# -----------------------------
# Controller class (updated)
# -----------------------------
class ODriveController(threading.Thread):
    def __init__(self, gear_ratio=100.0, max_torque=0.4, max_vel=60.0, name=None):
        super().__init__(daemon=True, name=name or "ODriveController")
        # run control flags
        self._stop_event = threading.Event()
        self._wake = threading.Event()
        self._estop_event = threading.Event()

        # locks
        self._axis_lock = threading.RLock()
        self.data_lock = threading.RLock()

        # hw handles & states
        self.odrv = None
        self.axis = None
        self.connected = False
        self.closed_loop_control = False
        self.error = False

        # offset handling
        self.is_offset = False
        self.offset = 0.0

        # mech/elect params
        self.gear_ratio = float(gear_ratio)
        self.max_torque = float(max_torque)
        self.max_vel = float(max_vel)  # deg/s (for trajectory limits)
        self.tor_coef = 0.708282
        self.Kt = 8.27 / 270.0

        # load/inertia (unchanged)
        self.link_mass = 1.125
        self.link_length = 0.7
        self.center_distance = 0.7 - 0.04
        self.motor_inertia = 0.000643
        self.const_inertia = (1.0/12.0) * self.link_mass * (self.link_length ** 2) + \
                              self.link_mass * (self.center_distance ** 2) + \
                              (self.gear_ratio ** 2) * self.motor_inertia

        self.hanger_mass = 0.26
        self.ext_load = 0.0
        self.hanger_distance = 0.7 - 0.04 - 0.06

        self.coul_friction = 0.0
        self.visc_friction = 0.00276 * (self.gear_ratio ** 2)

        # control gains and setpoints
        self.Kp = 1.0
        self.Kd = 0.5
        self.pos_set = -90.0
        self.vel_set = 0.0
        self.acc_set = 0.0

        self.ctrl_bandwidth = 2000
        self.enc_bandwidth = 1000

        # runtime state
        self.pos = 0.0
        self.vel = 0.0
        self.torque_set = 0.0

        # data buffer
        self.data = deque(maxlen=800)
        self.logging_enabled = True

        # trajectory object (None or Trajectory instance)
        self._trajectory = None
        self._traj_start_time = None

        self._running = False

    # -----------------------------
    # Trajectory management
    # -----------------------------
    def start_trajectory(self, traj):
        """Start following a trajectory (Trajectory instance). Thread-safe."""
        if not isinstance(traj, Trajectory):
            raise TypeError("traj must be Trajectory instance")
        with self.data_lock:
            self._trajectory = traj
            self._traj_start_time = time.time()
            # initialize setpoints to trajectory start sample
            pos0, vel0, acc0 = traj.sample(0.0)
            self.pos_set = pos0
            self.vel_set = vel0
            self.acc_set = acc0
        logger.info("Started trajectory: %r", traj)
        self._wake.set()

    def cancel_trajectory(self):
        """Cancel current trajectory (stops following; keeps current pos_set)."""
        with self.data_lock:
            self._trajectory = None
            self._traj_start_time = None
            self.acc_set = 0.0
            self.vel_set = 0.0
        logger.info("Trajectory canceled.")
        self._wake.set()

    def get_trajectory(self):
        with self.data_lock:
            return self._trajectory

    # -----------------------------
    # Move: now uses trajectory planning
    # -----------------------------
    def apply_move(self, target_deg, move_time_s=1.0, accel_fraction=0.3):
        """
        Called by GUI when 'Move' pressed.
        Creates a TrapezoidalTrajectory from current position to target, and starts it.
        vel_set and acc_set will be updated automatically by the trajectory during run().
        - target_deg: desired final position (degrees)
        - move_time_s: nominal move duration (seconds)
        - accel_fraction: fraction of T used for accel and decel (default 0.3)
        """
        with self.data_lock:
            start_pos = self.pos  # current measured pos (deg)
        traj = TrapezoidalTrajectory(start_pos, target_deg, move_time_s,
                                     accel_fraction=accel_fraction,
                                     vmax_limit=self.max_vel)
        # start trajectory
        self.start_trajectory(traj)

    # -----------------------------
    # dynamics & control (unchanged)
    # -----------------------------
    def dynamic_calculation(self, pos_deg, vel_deg_s):
        m = self.link_mass + self.hanger_mass + self.ext_load
        lc = (self.center_distance * self.link_mass + self.hanger_distance * (self.hanger_mass + self.ext_load)) / max(m, 1e-9)
        Ic = self.const_inertia + (self.hanger_mass + self.ext_load) * (self.hanger_distance ** 2)

        # convert to radians
        q = math.radians(pos_deg)
        qdot = math.radians(vel_deg_s)
        q_d = math.radians(self.pos_set)
        qdot_d = math.radians(self.vel_set)
        qddot_d = math.radians(self.acc_set)

        ep = q_d - q
        ev = qdot_d - qdot
        D = self.visc_friction

        tor = Ic * (qddot_d + (self.Kp * ep + self.Kd * ev)) + m * G * lc * math.cos(q) + D * qdot

        motor_torque_command = tor / (self.tor_coef * self.gear_ratio)
        motor_torque_command = max(min(motor_torque_command, self.max_torque), -self.max_torque)
        return motor_torque_command

    # -----------------------------
    # other methods unchanged (connect/clear_error/enter_closed_loop/return_idle/etc.)
    # For brevity, include representative ones; you can reuse the previously provided implementation.
    # -----------------------------
    def _safe_get(self, obj, *attrs, default=None):
        try:
            cur = obj
            for a in attrs:
                cur = getattr(cur, a)
            return cur
        except Exception:
            return default

    def connect(self, timeout=5.0):
        if odrive is None:
            logger.error("odrive module not available.")
            return False
        with self._axis_lock:
            try:
                logger.info("Connecting to ODrive...")
                self.odrv = odrive.find_any(timeout=timeout)
                if self.odrv is None:
                    logger.warning("ODrive not found.")
                    self.connected = False
                    return False
                self.axis = self.odrv.axis0
                self.connected = True
                self.error = False
                logger.info("Connected to ODrive.")
                return True
            except Exception as e:
                logger.exception("Connect failed: %s", e)
                self.connected = False
                self.error = True
                return False

    def set_offset(self):
        self.offset = self.axis.encoder.pos_estimate
        self.is_offset = True

    def enter_closed_loop(self):
        with self._axis_lock:
            if not self.axis:
                logger.warning("enter_closed_loop: no axis")
                return False
            try:
                # clear errors first
                try:
                    for attr in ("controller", "encoder", "motor"):
                        obj = getattr(self.axis, attr, None)
                        if obj is not None and hasattr(obj, "error"):
                            obj.error = 0
                    if hasattr(self.axis, "error"):
                        self.axis.error = 0
                except Exception:
                    pass
                self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                self.closed_loop_control = True
                logger.info("Requested CLOSED_LOOP_CONTROL.")
                return True
            except Exception:
                logger.exception("enter_closed_loop failed")
                self.closed_loop_control = False
                return False

    def return_idle(self):
        with self._axis_lock:
            try:
                if self.axis and hasattr(self.axis, "controller"):
                    self.axis.controller.input_torque = 0.0
                if self.axis:
                    self.axis.requested_state = AXIS_STATE_IDLE
                self.closed_loop_control = False
                logger.info("Requested IDLE.")
            except Exception:
                logger.exception("return_idle failed")

    def is_controlable(self):
        if not (self.connected and self.closed_loop_control and self.is_offset and not self._estop_event.is_set()):
            return False
        state = self._safe_get(self.axis, "current_state", default=None)
        return state == AXIS_STATE_CLOSED_LOOP_CONTROL

    def shutdown(self, wait_timeout=1.0):
        logger.info("Shutdown requested.")
        with self._axis_lock:
            try:
                if self.axis and hasattr(self.axis, "controller"):
                    self.axis.controller.input_torque = 0.0
                if self.axis:
                    self.axis.requested_state = AXIS_STATE_IDLE
            except Exception:
                logger.exception("Error during shutdown hardware ops")
        self._stop_event.set()
        self._wake.set()
        self.join(timeout=wait_timeout)
        try:
            self.disconnect()
        except Exception:
            pass
        logger.info("Shutdown complete.")

    def disconnect(self):
        with self._axis_lock:
            try:
                if self.axis is not None:
                    try:
                        self.axis.controller.input_torque = 0.0
                    except Exception:
                        pass
                    try:
                        self.axis.requested_state = AXIS_STATE_IDLE
                    except Exception:
                        pass
                self.axis = None
                self.odrv = None
                self.connected = False
                self.closed_loop_control = False
                logger.info("Disconnected.")
            except Exception:
                logger.exception("Disconnect error")

    # -----------------------------
    # Thread run loop: updated to follow trajectory setpoints when active
    # -----------------------------
    def run(self):
        logger.info("Controller thread starting.")
        self._running = True
        try:
            while not self._stop_event.is_set():
                # ensure connected
                if not self.connected:
                    self.connect(timeout=3.0)
                    if not self.connected:
                        self._wake.wait(timeout=1.0)
                        self._wake.clear()
                        continue

                # emergency stop handling
                if self._estop_event.is_set():
                    with self._axis_lock:
                        try:
                            if self.axis and hasattr(self.axis, "controller"):
                                self.axis.controller.input_torque = 0.0
                        except Exception:
                            pass
                    self._wake.wait(timeout=0.2)
                    self._wake.clear()
                    continue

                # Read sensors
                try:
                    with self._axis_lock:
                        if self.axis:
                            pos_est = self._safe_get(self.axis, "encoder", "pos_estimate", default=0.0)
                            vel_est = self._safe_get(self.axis, "encoder", "vel_estimate", default=0.0)
                            pos_deg = (float(pos_est) - float(self.offset)) * 360.0 / self.gear_ratio - 90.0
                            vel_deg_s = float(vel_est) * 360.0 / self.gear_ratio
                            Iq_set = self._safe_get(self.axis, "motor", "current_control", "Iq_setpoint", default=0.0)
                            try:
                                measured_torque = float(Iq_set) * self.Kt * self.gear_ratio * self.tor_coef
                            except Exception:
                                measured_torque = 0.0
                        else:
                            pos_deg = self.pos
                            vel_deg_s = self.vel
                            measured_torque = 0.0

                    with self.data_lock:
                        self.pos = pos_deg
                        self.vel = vel_deg_s
                        if self.logging_enabled:
                            self.data.append((time.time(), self.pos, self.vel, measured_torque))

                except Exception:
                    logger.exception("Sensor read error")
                    self.connected = False
                    self.closed_loop_control = False
                    self.error = True
                    self._wake.wait(timeout=1.0)
                    self._wake.clear()
                    continue

                # Trajectory following (if a trajectory is active)
                traj = None
                traj_start = None
                with self.data_lock:
                    traj = self._trajectory
                    traj_start = self._traj_start_time

                if traj is not None and traj_start is not None:
                    elapsed = time.time() - traj_start
                    # sample and apply setpoints
                    pos_t, vel_t, acc_t = traj.sample(elapsed)
                    with self.data_lock:
                        self.pos_set = pos_t
                        self.vel_set = vel_t
                        self.acc_set = acc_t
                    # if finished, clear trajectory
                    if elapsed >= traj.duration:
                        logger.info("Trajectory completed.")
                        with self.data_lock:
                            self._trajectory = None
                            self._traj_start_time = None
                            # keep final setpoints (pos_set = end), but zero vel/acc
                            self.pos_set = traj.end
                            self.vel_set = 0.0
                            self.acc_set = 0.0

                # Control output (if controlable)
                if self.is_controlable():
                    try:
                        with self.data_lock:
                            pos_copy = self.pos
                            vel_copy = self.vel
                        tcmd = self.dynamic_calculation(pos_copy, vel_copy)
                        with self._axis_lock:
                            if self.axis and hasattr(self.axis, "controller"):
                                self.axis.controller.input_torque = float(tcmd)
                                self.torque_set = float(tcmd)
                    except Exception:
                        logger.exception("Control write failed")
                        self.error = True
                        time.sleep(0.02)
                else:
                    # ensure no torque commanded
                    with self._axis_lock:
                        try:
                            if self.axis and hasattr(self.axis, "controller"):
                                self.axis.controller.input_torque = 0.0
                                self.torque_set = 0.0
                        except Exception:
                            pass

                # wait for next iteration or wake
                self._wake.wait(timeout=0.01)
                self._wake.clear()

        finally:
            # cleanup
            with self._axis_lock:
                try:
                    if self.axis and hasattr(self.axis, "controller"):
                        self.axis.controller.input_torque = 0.0
                    if self.axis:
                        self.axis.requested_state = AXIS_STATE_IDLE
                except Exception:
                    logger.exception("Cleanup error")
            self._running = False
            logger.info("Controller thread exiting.")