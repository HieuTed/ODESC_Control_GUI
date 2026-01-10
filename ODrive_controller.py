# odrive_controller.py

import threading
import time
import odrive
import math
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE
from Trajectory import TrapezoidalTrajectory as trapTraj
from collections import deque

CLOSED_LOOP_CONTROL = AXIS_STATE_CLOSED_LOOP_CONTROL
IDLE = AXIS_STATE_IDLE

gear_ratio = 100.0
g = 9.81
pi = math.pi

class ODriveThread(threading.Thread):
    def __init__(self):
        super().__init__()
        
        # running state
        self.connected = False
        self.closed_loop_control = False
        # self.running = True  
        # self.estop = False
        self.isOffset = False
        self.error = False
        self.data_lock = threading.Lock()  
        self.traj = trapTraj()  
        self._stop_event = threading.Event()
        self._estop_event = threading.Event() 

        # Components
        self.odrv = None
        self.axis = None
        self.max_torque = 0.4 # Giữ hằng số này trong controller nếu nó là giới hạn vật lý/cài đặt
        self.max_vel = 60.0 # rad/s
        self.tor_coef = 0.708282
        self.Kt = 8.27/270
        self.offset = 0.0

        #Load
        self.link_mass = 1.125
        self.link_length = 0.7
        self.center_distance = 0.7/2 - 0.04
        self.motor_inertia = 0.000643
        self.const_inertia = 1/12 * self.link_mass * (self.link_length ** 2) + self.link_mass * (self.center_distance ** 2) + gear_ratio**2 * self.motor_inertia

        self.hanger_mass = 0.26
        self.ext_load = 0.0
        self.hanger_distance = 0.7 - 0.04 - 0.06

        self.m = self.link_mass + self.hanger_mass + self.ext_load
        self.lc = (self.center_distance * self.link_mass + self.hanger_distance * (self.hanger_mass + self.ext_load))/self.m
        self.Ic = self.const_inertia + (self.hanger_mass + self.ext_load) * (self.hanger_distance **2)

        self.coul_friction = 0.0
        self.visc_friction = 0.00276 * gear_ratio**2

        # inputs
        self.Kp = 1.0
        self.Kd = 0.5
        self.torque_set = 0.0
        self.pos_set = - 90.0
        self.vel_set = 0.0    
        self.acc_set = 0.0    
        self.ctrl_bandwidth = 2000
        self.enc_bandwidth = 1000

        # show
        self.pos = 0.0
        self.vel = 0.0
        # self.data = []
        self.data = deque(maxlen=800)
        self.t_ref = math.inf

    def connect(self):
        try:
            print("Connecting to Odrive...")
            self.odrv = odrive.find_any(timeout=10) # Add timeout to prevent hang
            if self.odrv:
                print(f"Connected to Odrive: {self.odrv.serial_number}")       
                self.axis = self.odrv.axis0
                self.connected = True
                self.error = False
            else:
                print("ODrive not found")
        except Exception as e:
            self.error = True
            print(f"Connection failed: {e}")

    def clear_error(self):
        self.axis.controller.error = 0
        self.axis.encoder.error = 0
        self.axis.motor.error = 0
        self.axis.error = 0
        self.error = False
        print("Errors cleared")

    def enter_closed_loop(self):
        self.clear_error()
        self.axis.requested_state = CLOSED_LOOP_CONTROL
        self.closed_loop_control = True
        print("Entered CLOSED_LOOP_CONTROL mode.")

    def return_IDLE(self):
        self.axis.controller.input_torque = 0
        self.axis.requested_state = IDLE
        self.closed_loop_control = False

    def is_controlable(self):
        #return self.connected and self.closed_loop_control and self.isOffset and not self.estop
        return self.connected and self.closed_loop_control and self.isOffset and not self._estop_event.is_set()
    
    def update_parameter(self, *parameters): #không cần lock 
        try:
            self.ctrl_bandwidth = parameters[0]
            self.enc_bandwidth = parameters[1]
            with self.data_lock:
                self.axis.motor.config.current_control_bandwidth = self.ctrl_bandwidth
                self.axis.encoder.config.bandwidth = self.enc_bandwidth
            
            self.ext_load = parameters[2]
            self.hanger_distance = parameters[3]
            self.coul_friction = parameters[4]
            self.visc_friction = parameters[5]
            self.Kp = parameters[6]
            self.Kd = parameters[7]

            self.m = self.link_mass + self.hanger_mass + self.ext_load
            self.lc = (self.center_distance * self.link_mass + self.hanger_distance * (self.hanger_mass + self.ext_load))/self.m
            self.Ic = self.const_inertia + (self.hanger_mass + self.ext_load) * (self.hanger_distance **2)

        except Exception as e:
            print("Parameter update error:", e)

    def get_state(self):
        if self.connected: return self.axis.current_state
        return None
    
    def emergency_stop(self):
        """Immediate torque -> 0 and prevent further torque writes until reset"""
        #self.estop = True
        self._estop_event.set()
        self.torque_set = 0.0
        try:
            if self.connected and self.axis is not None:
                self.axis.controller.input_torque = 0
        except Exception:
            pass

    def get_data(self): #cần sử dụng để kiểm tra chế độ của ODrive - Sử dụng ở GUI
        with self.data_lock:
            return list(self.data)
    
    def set_offset(self): #Sử dụng ở GUI
        try:
            self.offset = self.axis.encoder.pos_estimate
            self.isOffset = True
        except Exception:
            pass

    def moveTo(self, newP, moveT):
        with self.data_lock:
            self.t_ref = time.time()
            self.traj.param_calc(self.pos, newP, moveT, self.max_vel)

    def reset(self):
        self.return_IDLE()
        self.isOffset = False
        #self.estop = False
        self._estop_event.clear()

    def stop(self):
        # self.running = False  
        self._stop_event.set()
        try:
            if self.axis is not None:
                self.axis.controller.input_torque = 0
                time.sleep(0.05)
                self.axis.requested_state = AXIS_STATE_IDLE  
                time.sleep(0.05)
        except Exception:
            pass

    def dynamic_calculation(self):
        m = self.m
        lc = self.lc
        Ic = self.Ic

        q = self.pos * (2 * pi) / 360
        qdot = self.vel * (2 * pi) / 360
        q_d = self.pos_set * (2 * pi) / 360
        qdot_d = self.vel_set * (2 * pi) / 360
        qddot_d = self.acc_set * (2 * pi) / 360

        Kp = self.Kp
        Kd = self.Kd
        ep = q - q_d
        ev = qdot - qdot_d
        D = self.visc_friction

        tor = Ic * (qddot_d - (Kp * ep + Kd * ev)) + m * g * lc * math.cos(q) + D*qdot 
        # Ff = self.coul_friction * math.copysign(1, qdot) if abs(qdot) > 0.01 else 0
        tor = (tor)/self.tor_coef/gear_ratio
        self.torque_set = max(min(tor, self.max_torque), -self.max_torque)

    def run(self):
        # while self.running:
        while not self._stop_event.is_set():
            t_start = time.perf_counter()
            try:
                if not self.connected:
                    self.connect()
                    if not self.connected: 
                        time.sleep(0.1)
                        continue

                # if self.estop:
                #     time.sleep(0.1)
                #     continue    

                # if self.isOffset and not self.closed_loop_control:
                #     self.enter_closed_loop()
                # else: 
                #     time.sleep(0.1)
                #     continue
                if self._estop_event.is_set():
                    self._stop_event.wait(0.1)
                    continue    

                # if self.isOffset and not self.closed_loop_control:
                #     self.enter_closed_loop()
                # else: 
                #     self._stop_event.wait(0.1)
                #     continue

                # Data collect
                with self.data_lock:
                    self.pos = (self.axis.encoder.pos_estimate - self.offset) * 360 / gear_ratio - 90
                    self.vel = self.axis.encoder.vel_estimate * 360 / gear_ratio
                    tor_set = self.axis.motor.current_control.Iq_setpoint * self.Kt * gear_ratio * self.tor_coef
                    self.data.append((time.time(), self.pos, self.vel, self.pos_set, self.vel_set, tor_set))
                    if len(self.data) > 800:
                        self.data = self.data[-800:]

                if self.is_controlable():
                    t_traj = time.time() - self.t_ref
                    with self.data_lock:
                        self.pos_set, self.vel_set, self.acc_set = self.traj.desired_state(t_traj)
                        self.dynamic_calculation()
                        self.axis.controller.input_torque = self.torque_set
                else: 
                    self.torque_set = 0.0
                
                # time.sleep(0.01)

            except Exception as e:
                print("ODrive error:", e)
                self.connected = False
                self.closed_loop_control = False
                self.error = True
                time.sleep(1)
            
            t_end = time.perf_counter()
            t_sleep = 0.01 - (t_end - t_start)
            if t_sleep > 0:
                self._stop_event.wait(timeout=t_sleep)

