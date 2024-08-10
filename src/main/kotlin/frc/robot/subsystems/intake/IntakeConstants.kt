package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object IntakeConstants {
    const val INTAKE_SPIN_POWER = 0.4
    const val INTAKE_CENTER_POWER = 0.4
    const val GEAR_RATIO = 45.62

    val INTAKE_ANGLE: Measure<Angle> = Units.Degree.zero()

    val REST_ANGLE: Measure<Angle> = Units.Degree.of(114.0)
}
