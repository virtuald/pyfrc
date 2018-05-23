"""
    .. note:: This motor modeling code was taken from the `SnobotSim <https://github.com/pjreiniger/SnobotSim>`_
              project, released under a MIT license by PJ Reiniger.
    
    
"""

from collections import namedtuple
import math
from typing import Optional



def makeTransmission(config, numMotors: int, gearReduction: float, efficiency: float):
    """
        This does things
    """
    
    # DcMotorModelConfig.FactoryParams factoryParams = new DcMotorModelConfig.FactoryParams(aMotor.mFactoryParams.mMotorType, aNumMotors,
    #             aGearReduction, aEfficiency, aMotor.mFactoryParams.mInverted, aMotor.mFactoryParams.mHasBrake);
    #     DcMotorModelConfig.MotorParams modifiedMotorParams = new DcMotorModelConfig.MotorParams(
    #             aMotor.mMotorParams.NOMINAL_VOLTAGE,
    #             aMotor.mMotorParams.FREE_SPEED_RPM / aGearReduction,
    #             aMotor.mMotorParams.FREE_CURRENT  * aNumMotors,
    #             aMotor.mMotorParams.STALL_TORQUE  * aNumMotors,
    #             aMotor.mMotorParams.STALL_CURRENT * aNumMotors,
    #             aMotor.mMotorParams.MOTOR_INERTIA * aNumMotors * aGearReduction * aGearReduction,
    #             aEfficiency * aNumMotors * aGearReduction)
    #
    #     return new DcMotorModelConfig(factoryParams, modifiedMotorParams)



class DcMotorModel:
    """
        Models the operation of a motor and keeps track of the motor current,
        position, velocity, and acceleration.
        
        
    """
    
    # Snobot sim adapted this from https://github.com/Team254/Sim-FRC-2015/blob/master/src/com/team254/frc2015/sim/DCMotor.java
    
    def __init__(self, motor_config: MotorModelConfig, *,
                       inertia: float=0,
                       ktScaler: float=1,
                       kt: Optional[float]=None, kv: Optional[float]=None,
                       resistance: Optional[float]=None, hasBrake: bool=False):
        self.config = motor_config
        
        self.position = 0
        self.velocity = 0
        self.current = 0
        self.acceleration = 0
        self.force = 0
        
        if kt is None:
            kt = (self.config.stallTorque / self.config.stallCurrent)
        
        if kv is None:
            kv = (self.config.freeSpeedRpm / self.config.nominalVoltage) * (2 * math.pi) / 60.0
        
        if resistance is None:
            resistance = self.config.nominalVoltage / self.config.stallCurrent
            
        print(kt, kv, resistance)
        
        self._hasBrake = hasBrake
        self._inertia = inertia
        self._ktScaler = ktScaler
        self._kt = kt * ktScaler
        self._kv = kv
        self._resistance = resistance
    
    # def addTransmission(self, numMotors: int, gearReduction: float, efficiency: float):
    #
    #     # XXX: this has to be external
    #
    #     self.config = MotorModelConfig(
    #         self.config.nominalVoltage,
    #         self.config.freeSpeedRpm / gearReduction,
    #         self.config.freeCurrent * numMotors,
    #         self.config.stallTorque * numMotors,
    #         self.config.stallCurrent * numMotors
    #     )
    #
    #     self._inertia *= numMotors * gearReduction * gearReduction
    #
    #     ktScaler = efficiency * numMotors * gearReduction
    #     self._kt = (self._kt / self._ktScaler) * ktScaler
    #     self._ktScaler = ktScaler
    
    def reset(self, position: float, velocity: float, current: float) -> None:
        """Reset the model to a specified state"""
        self.position = position
        self.velocity = velocity
        self.current = current
        self.acceleration = 0
        self.force = 0
    
    def step(self, appliedVoltage: float, load: float, externalTorque: float, tm_diff: float) -> None:
        """Simulate applying a given voltage and load for a specified period of
        time.
        
        :param appliedVoltage:  Voltage applied to the motor (Volts)
        :param load:            Load applied to the motor (kg*m^2)
        :param externalTorque:  (kg*m^2)
        :param tm_diff:         How long the input is applied (seconds)
        """
        
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
        

        
        # F=MA
        # .. so we just need acceration and this is fine
         
        
        # Vapp = kv * velocity + ka * acceleration + Vintercept
        
        # me..
        #
        
        # .. their Vintercept is ~1.26v .. kv = 0.81 .. ka = 0.21 or 0.1 (real)
        # .. voltage required to generate enough torque to overcome constant
        #    steady-state frictional effects

        if self._hasBrake and abs(appliedVoltage) < 0.000001:
            # TODO: this isn't how this works, do this more realistically?
            self.acceleration = 0
            self.velocity = 0
            self.current = 0
            self.force = 0
        else:
            #appliedVoltage = math.copysign(max(abs(appliedVoltage) - 1.2, 0), appliedVoltage)
            
            load += self._inertia
            #print(appliedVoltage, load)
            
            #print(self._resistance * load, self._kt)
            #print(self._kt / (self._resistance * load))
            print(appliedVoltage, self.velocity, self._kv)
            print(appliedVoltage - self.velocity / self._kv)
            
            self.acceleration = (appliedVoltage - self.velocity / self._kv) * self._kt \
                    / (self._resistance * load) + externalTorque / load
            #sprint(externalTorque, load)
            self.force = self.acceleration * load
            print(self.force)
            self.velocity += self.acceleration * tm_diff
            self.position += self.velocity * tm_diff # + .5 * self.acceleration * tm_diff * tm_diff
            self.current = load * self.acceleration * math.copysign(1, appliedVoltage) / self._kt
            print(self.current * self._kt)

        # Oblarg's model:
        #
        # Vapp = kw + I*R
        # Vapp = Vemf + Vwindings
        # Vemf = kw
        # Vwindings = I*R
        # Vemf = kv * velocity
        # Vwindings = ka * acceleration
        
        # Vapp = kv * velocity + ka * acceleration
        
        # kv = Vmax / velocity_max
        # velocity_max = (wfree * pi * wheel_diameter) / rgearing
        
        # ka = Vmax / acceleration_max
        # acceleration_max = 2*n*tstall*rgearing / (wheel_diameter * Mrobot)

class SimpleMotorSimulator:
    def __init__(self, maxSpeed):
        self.maxSpeed = maxSpeed
        self.position = 0
    
    def compute(self, motor_pct: float, tm_diff: float) -> None:
        """
            :param motor_pct: Motor value in range [-1..1]
            :param tm_diff:   Amount of time to compute simulation for
        """
        self.velocity = self.maxSpeed * motor_pct
        self.position += self.velocity * tm_diff

class BaseDcMotorSimulator:
    def __init__(self, model: DcMotorModel, scale: float=1):
        self.model = model
        self.scale = scale
    
    @property
    def acceleration(self) -> float:
        return self.model.acceleration * self.scale
    
    @property
    def position(self) -> float:
        return self.model.position * self.scale
    
    @property
    def velocity(self) -> float:
        return self.model.velocity * self.scale



class RotationalLoadDcMotorSim(BaseDcMotorSimulator):
    """
        Simulator for a rotational load, such as a spinning wheel
    """

    GRAVITY = 9.8
    
    def __init__(self):
        """
            :param model:                  Motor model
            :param load:                   Load to move (kg*m^2)
            :param scale:       Scale position/velocity/accel by this value
        
            :param armCenterOfMass:        The location of the center of mass, in meters
            :param armMass:                The mass of the arm, in kg
            :param constantAssistTorque:   torque provided constantly over the entire range of motion, in
                                           N*m
            :param overCenterAssistTorque: torque provided varying with ``sin`` of angle (same effect as
                                           gravity) due to over-center, in N*m
        """
        
        mConfig = aConfig

        mSpeedController = aSpeedController
        mArmInertia = mConfig.mArmMass * mConfig.mArmCenterOfMass * mConfig.mArmCenterOfMass
        mGravityBasedTorqueFactor = mConfig.mArmMass * mConfig.mArmCenterOfMass * self.GRAVITY
        mConstantAssistTorque = mConfig.mConstantAssistTorque
        mOverCenterAssistTorque = mConfig.mOverCenterAssistTorque
    
    def compute(self, motor_pct: float, tm_diff: float) -> None:
        """
            :param motor_pct: Motor value in range [-1..1]
            :param tm_diff:   Amount of time to compute simulation for
        """
        position = self.position # speedController.getPosition
        gravityTorque = self.gravityBasedTorqueFactor * math.sin(position)
        gravityTorque += mConstantAssistTorque
        gravityTorque += mOverCenterAssistTorque * math.sin(position)
        
        inVolts = motor_pct * self.model.config.nominalVoltage
        self.model.step(inVolts, self.mArmInertia, gravityTorque, tm_diff)
    
class StaticLoadDcMotorSim(BaseDcMotorSimulator):
    """
        
    """
    
    def __init__(self, model: DcMotorModel, load: float, scale: float=1.0):
        """
            :param model:    Motor model
            :param load:     Load to move (kg*m^2)
            :param scale:    Scale position/velocity/accel by this value
        """
        super().__init__(model, scale=scale)
        self.load = load
    
    def compute(self, motor_pct: float, tm_diff: float) -> None:
        """
            :param motor_pct: Motor value in range [-1..1]
            :param tm_diff:   Amount of time to compute simulation for
        """
        self.model.step(motor_pct * self.model.config.nominalVoltage, self.load, 0, tm_diff)

class GravityLoadDcMotorSim(StaticLoadDcMotorSim):
    """
        Simulates a motor moving a load vertically
    """
    
    GRAVITY = 9.8
     
    def compute(self, motor_pct: float, tm_diff: float) -> None:
        """
            :param motor_pct: Motor value in range [-1..1]
            :param tm_diff:   Amount of time to compute simulation for
        """
        extraAcceleration = -self.GRAVITY
        self.model.step(motor_pct * self.model.config.nominalVoltage, self.load, extraAcceleration, tm_diff)
