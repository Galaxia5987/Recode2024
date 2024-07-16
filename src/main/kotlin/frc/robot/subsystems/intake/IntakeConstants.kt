package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object IntakeConstants {
    const val intakeSpinPower = 0.4
    const val intakeCenterPower = 0.4

    val intakeAngle: Measure<Angle> = Units.Degree.zero()
    val restAngle: Measure<Angle> = Units.Degree.of(114.0)
}
