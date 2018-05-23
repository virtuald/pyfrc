
from collections import namedtuple
import math

from .units import units

MotorModelConfig = namedtuple('MotorModelConfig',[
    'name',
    'nominalVoltage',
    'freeSpeed',
    'freeCurrent',
    'stallTorque',
    'stallCurrent',
    'R',
    'Kv',
    'Kt',
])
MotorModelConfig.__doc__ = """
    Configuration parameters useful for simulating a motor. Typically these
    parameters can be obtained from the manufacturer via a data sheet or other
    specification.
    
    RobotPy contains MotorModelConfig objects for many motors that are commonly
    used in FRC. If you find that we're missing a motor you care about, please
    file a bug report and let us know!
    
    .. note:: The motor configurations that come with pyfrc are defined using the
              pint units library. See :ref:`units`
"""
MotorModelConfig.name.__doc__ = 'Descriptive name of motor'
MotorModelConfig.nominalVoltage.__doc__ = 'Nominal voltage for the motor'
MotorModelConfig.freeSpeed.__doc__   = 'No-load motor speed (``1 / [time]``)'
MotorModelConfig.freeCurrent.__doc__    = 'No-load motor current'
MotorModelConfig.stallTorque.__doc__    = 'Stall torque (``[length]**2 * [mass] / [time]**2``)'
MotorModelConfig.stallCurrent.__doc__   = 'Stall current'
MotorModelConfig.R.__doc__   = 'Resistance of motor (ohms)'
MotorModelConfig.Kv.__doc__   = 'Motor velocity constant (``1 / [time] / volt``)'
MotorModelConfig.Kt.__doc__   = 'Motor torque constant (``[length] ** 2 * [mass] / [current] / [time] ** 2``)'

def define_motor_config(name, nominalVoltage, freeSpeed, freeCurrent,
                        stallTorque, stallCurrent):
    """
        Helper function that computes motor configuration based on other
        configuration parameters
    """
    # convert these all to consistent units in case a user tries to use this
    nominalVoltage = units.volts.from_(nominalVoltage)
    freeSpeed = units.cpm.from_(freeSpeed)
    freeCurrent = units.amps.from_(freeCurrent)
    stallTorque = units.N_m.from_(stallTorque)
    stallCurrent = units.amps.from_(stallCurrent)
    R = units.ohms.from_(nominalVoltage / stallCurrent)
    Kv = freeSpeed * (2 * math.pi) / (nominalVoltage - R * freeCurrent)
    Kt = stallTorque / stallCurrent
    return MotorModelConfig(name, nominalVoltage, freeSpeed, freeCurrent,
                            stallTorque, stallCurrent, R, Kv, Kt)

NOMINAL_VOLTAGE = 12 * units.volts

#: Motor configuration for CIM
MOTOR_CFG_CIM = define_motor_config(
    'CIM', NOMINAL_VOLTAGE,
    5310 * units.cpm,
    2.7 * units.amps,
    2.42 * units.N_m,
    133 * units.amps
)

#: Motor configuration for Mini CIM
MOTOR_CFG_MINI_CIM = define_motor_config(
    'MiniCIM', NOMINAL_VOLTAGE,
    5840 * units.cpm,
    3.0 * units.amps,
    1.41 * units.N_m,
    89.0 * units.amps
)

#: Motor configuration for Bag Motor
MOTOR_CFG_BAG = define_motor_config(
    'Bag', NOMINAL_VOLTAGE,
    13180 * units.cpm,
    1.8 * units.amps,
    0.43 * units.N_m,
    53.0 * units.amps
)

#: Motor configuration for 775 Pro
MOTOR_CFG_775PRO = define_motor_config(
    '775Pro', NOMINAL_VOLTAGE,
    18730 * units.cpm,
    0.7 * units.amps,
    0.71 * units.N_m,
    134 * units.amps
)

#: Motor configuration for Andymark RS 775-125
MOTOR_CFG_775_125 = define_motor_config(
    'RS775-125', NOMINAL_VOLTAGE,
    5800 * units.cpm,
    1.6 * units.amps,
    0.28 * units.N_m,
    18.0 * units.amps
)

#: Motor configuration for Banebots RS 775
MOTOR_CFG_BB_RS775 = define_motor_config(
    'RS775', NOMINAL_VOLTAGE,
    13050 * units.cpm,
    2.7 * units.amps,
    0.72 * units.N_m,
    97.0 * units.amps
)

#: Motor configuration for Andymark 9015
MOTOR_CFG_AM_9015 = define_motor_config(
    'AM-9015', NOMINAL_VOLTAGE,
    14270 * units.cpm,
    3.7 * units.amps,
    0.36 * units.N_m,
    71.0 * units.amps
)

#: Motor configuration for Banebots RS 550
MOTOR_CFG_BB_RS550 = define_motor_config(
    'RS550', NOMINAL_VOLTAGE,
    19000 * units.cpm,
    0.4 * units.amps,
    0.38 * units.N_m,
    84.0 * units.amps
)
