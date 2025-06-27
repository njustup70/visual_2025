#增量式pid控制器
class pid_increase_t:
    def __init__(self, kp, ki, kd,min,max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.min = min
        self.max = max
    def set_target(self, target):
        self.target = target
    def update(self, current_value):
        error = self.target - current_value
        self.integral += error
        output= self.kp * error +  self.ki*self.previous_error-2*self.kd*(current_value-self.previous_error)
        self.previous_error = error
        if output > self.max:
            output = self.max
        elif output < self.min:
            output = self.min
        return output