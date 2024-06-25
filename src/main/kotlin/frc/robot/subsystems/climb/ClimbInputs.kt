package frc.robot.subsystems.climb

import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage

class ClimbInputs {
    var stopperSetpoint : MutableMeasure<Angle> = Units.Degree.of(0.0).mutableCopy()
    var appliedVoltage : MutableMeasure<Voltage> = Units.Volts.of(0.0).mutableCopy()
}