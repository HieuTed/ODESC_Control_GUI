from abc import ABC, abstractmethod
import math

class TrajectoryBase(ABC):
    def __init__(self):
        # Khởi tạo các biến lưu trữ tham số đã tính toán
        self.start_p = -90.0
        self.end_p = -90.0
        self.maxVel = math.inf
        self.direction = 1.0
        self.total_time = math.inf 
        
    @abstractmethod
    def param_calc(self, start_p, end_p, max_v):
        """
        Method này chạy NẶNG, chứa logic phức tạp (căn bậc 2, giải phương trình...).
        Chỉ gọi 1 lần khi có lệnh Move mới.
        """
        pass

    @abstractmethod
    def desired_state(self, t):
        """
        Method này chạy NHẸ (chỉ cộng trừ nhân chia đơn giản).
        Gọi liên tục trong vòng lặp timer (Real-time).
        Output: pos, vel, acc
        """
        pass
    def reset(self):
        self.start_p = -90.0
        self.end_p = -90.0
        self.total_time = math.inf
        self.direction = 1.0 

class TrapezoidalTrajectory(TrajectoryBase):
    def __init__(self):
        super().__init__()
        # Các biến riêng của hình thang
        self.j_peak = 4   # deg/s^3
        self.accel = 10.0  # deg/s^2 - gia tốc cố định
        self.v_peak = 0.0 # deg/s
        self.t_acc = 0.0  # Thời gian tăng tốc
        self.t_dec = 0.0  # Thời điểm bắt đầu giảm tốc

    def param_calc(self, start_p, end_p, max_v):
        self.start_p = start_p
        self.end_p = end_p
        distance = end_p - start_p
        self.direction = 1.0 if distance >= 0 else -1.0
        abs_dist = abs(distance)
        
        if abs_dist < 0.005:
            self.v_peak = 0.0
            self.t_acc = 0.0
            self.total_time = 0.0
            self.t_dec = 0.0
            return
        
        # Tính thời gian tăng tốc
        self.t_acc = max_v / self.accel
        d_acc = 0.5 * self.accel * self.t_acc ** 2
        
        if abs_dist <= 2 * d_acc:
            # Không có giai đoạn tốc độ đều
            self.v_peak = math.sqrt(abs_dist * self.accel)
            self.t_acc = self.v_peak / self.accel
            self.total_time = 2 * self.t_acc
            self.t_dec = self.t_acc
        else:
            # Có giai đoạn tốc độ đều
            self.v_peak = max_v
            d_const = abs_dist - 2 * d_acc
            t_const = d_const / max_v
            self.total_time = 2 * self.t_acc + t_const
            self.t_dec = self.t_acc + t_const


    def desired_state(self, t):
        # Xử lý ngoài phạm vi

        if t <= 0: return self.start_p, 0.0, 0.0
        if t >= self.total_time or self.total_time == math.inf: return self.end_p, 0.0, 0.0

        pos = 0.0
        vel = 0.0
        acc = 0.0   

        if t < self.t_acc: # Giai đoạn 1: Tăng tốc
            pos = 0.5 * self.accel * t * t
            vel = self.accel * t
            acc = self.accel
            
        elif t < self.t_dec: # Giai đoạn 2: Tốc độ đều
            dt = t - self.t_acc
            pos = 0.5 * self.accel * self.t_acc**2 + self.v_peak * dt
            vel = self.v_peak
            acc = 0.0
            
        else: # Giai đoạn 3: Giảm tốc
            t_rem = self.total_time - t
            dist_rem = 0.5 * self.accel * t_rem * t_rem
            pos = abs(self.end_p - self.start_p) - dist_rem
            vel = self.accel * t_rem
            acc = -self.accel

        # Kết hợp hướng
        final_p = self.start_p + pos * self.direction
        final_v = vel * self.direction
        final_a = acc * self.direction
        
        return final_p, final_v, final_a
    
class CubicTrajectory(TrajectoryBase):
    def __init__(self):
        super().__init__()
        # Hệ số phương trình: q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        self.a0 = 0
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0

    def param_calc(self, start_p, end_p, max_v):
        
        self.start_p = start_p
        self.end_p = end_p
        dist = end_p - start_p
        abs_dist = abs(dist)
        
        # Tính thời gian dựa trên max_v, giả sử thời gian = abs_dist / max_v * 2 để smooth và không vượt quá max_v quá nhiều
        self.total_time = abs_dist / max_v * 2.0 if max_v > 0 else 1.0
        
        T = self.total_time
        self.a0 = start_p
        self.a1 = 0
        self.a2 = 3 * dist / (T**2)
        self.a3 = -2 * dist / (T**3)

    def desired_state(self, t):
        if t <= 0: return self.start_p, 0.0, 0.0
        if t >= self.total_time or self.total_time == math.inf: return self.end_p, 0.0, 0.0
        
        t2 = t*t
        t3 = t2*t
        
        return pos, vel, acc
    
class QuinticTrajectory(TrajectoryBase):
    def __init__(self):
        super().__init__()
        # Hệ số phương trình bậc 5: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        self.a0 = 0
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0
        self.a5 = 0

    def param_calc(self, start_p, end_p, max_v):
        self.start_p = start_p
        self.end_p = end_p
        dist = end_p - start_p
        
        # Tính thời gian dựa trên max_v, giả sử thời gian = abs(dist) / max_v * 2.5 để smooth
        abs_dist = abs(dist)
        self.total_time = abs_dist / max_v * 2.5 if max_v > 0 else 1.0
        
        T = self.total_time
        T2 = T * T
        T3 = T2 * T
        T4 = T3 * T
        T5 = T4 * T
        
        self.a0 = start_p
        self.a1 = 0
        self.a2 = 0
        self.a3 = 10 * dist / T3
        self.a4 = -15 * dist / T4
        self.a5 = 6 * dist / T5

    def desired_state(self, t):
        if t <= 0: return self.start_p, 0.0, 0.0
        if t >= self.total_time or self.total_time == math.inf: return self.end_p, 0.0, 0.0
        
        t2 = t * t
        t3 = t2 * t
        t4 = t3 * t
        t5 = t4 * t
        
        pos = self.a0 + self.a1*t + self.a2*t2 + self.a3*t3 + self.a4*t4 + self.a5*t5
        vel = self.a1 + 2*self.a2*t + 3*self.a3*t2 + 4*self.a4*t3 + 5*self.a5*t4
        acc = 2*self.a2 + 6*self.a3*t + 12*self.a4*t2 + 20*self.a5*t3
        
        return pos, vel, acc