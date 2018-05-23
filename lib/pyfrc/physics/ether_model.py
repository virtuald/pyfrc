

class Model:
    
    def __init__(self):
        self.drivetrain_efficiency = 0
        self.gear_ratio = 0
        self.number_of_motors = 0
        self.vehicle_force = 0
    
    def _compute_acceleration(self, velocity):
        motor_torque = self.motor_stall_torque * (1 - velocity / self.Vfree)
        
        wheel_torque = self.drivetrain_efficiency * motor_torque * self.gear_ratio
        
        rolling_resistance_losses = self.rolling_resistance + self.rolling_resistance_prime * velocity
        
        self.vehicle_force = wheel_torque / self.wheel_radius * self.number_of_motors - rolling_resistance_losses
        if self.vehicle_force < 0:
            self.vehicle_force = 0
        
        if self.vehicle_force > self.mass * self.us:
            self.slipping = True
        elif self.vehicle_force < self.mass * self.uk:
            self.slipping = False
        
        if self.slipping:
            self.vehicle_force = self.mass * self.uk
        
        return self.vehicle_force / self.mass
    
    def compute(self, tm_diff):
        
        # Heun's method (taken from Ether's drivetrain calculator)
        # -> yn+1 = yn + (h/2) (f(xn, yn) + f(xn + h, yn +  h f(xn, yn)))
        
        a0 = self.acceleration
        v0 = self.velocity
        
        v1 = v0 + a0 * tm_diff
        a1 = self._compute_acceleration(v0)
        
        v1 = v0 + (a0 + a1) * 0.5 * tm_diff
        a1 = self._compute_acceleration(v1)
        self.position += (v0 + v1) * 0.5 * tm_diff
        
        self.velocity = v1
        self.acceleration = a1
