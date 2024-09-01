package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

object IntakeConstants {
    const val INTAKE_SPIN_POWER = -0.4
    const val INTAKE_CENTER_POWER = -0.4
    const val GEAR_RATIO = 55.56

    val GAINS by lazy {
        selectGainsBasedOnMode(
            Gains (
                21.0,
                kD= 0.1
            ),
            Gains(
                10.1 / 360.0
            )
        )
    }

    val INTAKE_ANGLE: Measure<Angle> = Units.Degree.zero()
    val REST_ANGLE: Measure<Angle> = Units.Degree.of(120.0)
}
