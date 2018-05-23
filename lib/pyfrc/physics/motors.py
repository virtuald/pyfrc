

class BaseMotorModel:
    
    velocity = 0
    acceleration = 0
    position = 0
    
    def __init__(self):
        pass
    
    def _compute_acceleration(self, velocity):
        raise NotImplementedError
    
    def compute(self, motor_pct: float, tm_diff: float) -> float:
        '''
            :param motor_pct: Percentage of power for motor in range [1..-1]
            :param tm_diff:   Time elapsed since this function was last called
            
            :returns: velocity
        '''
        
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

# oblarg model is the same as above, just load is 0

class Motor254(BaseMotorModel):
    
    def __init__(self):
        super().__init__()
    
    def _compute_acceleration(self, velocity):
        load += self._inertia
        return = (appliedVoltage - velocity / self._kv) * self._kt \
                / (self._resistance * load) + externalTorque / load
