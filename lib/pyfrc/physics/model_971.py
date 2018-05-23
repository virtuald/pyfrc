
#
# Using the 971-style first order system model.
#
# V = I * R + Kv * w
# torque = Kt * I
#
# V = torque / Kt * R + Kv * w
# torque = J * dw/dt + external_torque
#
# dw/dt = (V - Kv * w) * Kt / (R * J) - external_torque / J
#


v = voltage percent

self.acceleration = gearing * Kt / (R * r * robot_mass) * v - gearing**2 * Kt / (Kv * R * r**2 * robot_mass) * self.velocity
self.velocity += self.acceleration * dt
