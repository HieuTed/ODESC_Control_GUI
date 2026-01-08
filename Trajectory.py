from abc import ABC, abstractmethod

class TrajectoryBase(ABC):
    def __init__(self):
        # Khởi tạo các biến lưu trữ tham số đã tính toán
        self.start_p = 0.0
        self.end_p = 0.0
        self.total_time = 0.0
        self.direction = 1.0
        
    @abstractmethod
    def param_calc(self, start_p, end_p, move_t, max_v):
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

class TrapezoidalTrajectory(TrajectoryBase):
    def __init__(self):
        super().__init__()
        # Các biến riêng của hình thang
        self.accel = 0.0
        self.v_peak = 0.0
        self.t_acc = 0.0  # Thời gian tăng tốc
        self.t_dec = 0.0  # Thời điểm bắt đầu giảm tốc

    def param_calc(self, start_p, end_p, move_t, max_v):
        self.start_p = start_p
        self.end_p = end_p
        distance = end_p - start_p
        self.direction = 1.0 if distance >= 0 else -1.0
        abs_dist = abs(distance)
        c = 1 - abs_dist / (move_t * max_v)

        if c < 0.1: 
            self.v_peak = max_v
            self.accel = (1-0.1)/0.1 * max_v**2/abs_dist 
            self.t_acc = self.v_peak / self.accel 
            self.total_time = self.t_acc / 0.1
            self.t_dec = self.total_time - self.t_acc

        elif c <= 0.35:
            self.total_time = float(move_t)
            self.v_peak = max_v
            self.t_acc = c * self.total_time
            self.t_dec = self.total_time - self.t_acc
            self.accel = self.v_peak / self.t_acc
        
        else:
            self.total_time = float(move_t)
            self.t_acc = 0.35 * self.total_time
            self.t_dec = self.total_time - self.t_acc
            self.accel = abs_dist / (self.total_time**2 * 0.35*(1-0.35))  
            self.v_peak = self.t_acc * self.accel


    def desired_state(self, t):
        # Xử lý ngoài phạm vi

        if t <= 0: return self.start_p, 0.0, 0.0
        if t >= self.total_time: return self.end_p, 0.0, 0.0

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

    def param_calc(self, start_p, end_p, move_t, max_v):
        # Lưu ý: Cubic cơ bản không đảm bảo max_v, nó chỉ đảm bảo đến đích đúng giờ.
        # max_v ở đây chỉ để giữ chuẩn interface (hoặc có thể dùng để check warning)
        
        self.start_p = start_p
        self.end_p = end_p
        self.total_time = float(move_t)
        
        # Giải hệ phương trình boundary conditions (start v=0, end v=0)
        # q(0) = start, q(T) = end
        # v(0) = 0, v(T) = 0
        
        T = self.total_time
        dist = end_p - start_p
        
        self.a0 = start_p
        self.a1 = 0
        self.a2 = 3 * dist / (T**2)
        self.a3 = -2 * dist / (T**3)

    def desired_state(self, t):
        if t <= 0: return self.start_p, 0.0, 0.0
        if t >= self.total_time: return self.end_p, 0.0, 0.0
        
        t2 = t*t
        t3 = t2*t
        
        pos = self.a0 + self.a1*t + self.a2*t2 + self.a3*t3
        vel = self.a1 + 2*self.a2*t + 3*self.a3*t2
        acc = 2*self.a2 + 6*self.a3*t
        
        return pos, vel, acc