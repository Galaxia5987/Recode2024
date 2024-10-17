package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

object IntakeConstants {
    const val INTAKE_SPIN_POWER = -0.65
    const val INTAKE_CENTER_POWER = -0.4
    const val GEAR_RATIO = 55.56

    val GAINS by lazy {
        selectGainsBasedOnMode(
            Gains (
                kP = 55.0,
                kI = 3.0,
                kD = 0.1,
            ),
            Gains(
                10.1 / 360.0
            )
        )
    }

    val INTAKE_ANGLE: Measure<Angle> = Units.Degree.of(20.0)
    val REST_ANGLE: Measure<Angle> = Units.Degree.of(126.2)
}
