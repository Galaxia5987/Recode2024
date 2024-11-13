package frc.robot.subsystems.conveyor

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Dimensionless
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val GEAR_RATIO = 1.0

val AT_SETPOINT_TOLERANCE: Dimensionless = Units.Percent.of(0.1)

val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.000_05)

val FEED_VELOCITY: AngularVelocity = Units.RotationsPerSecond.of(70.0)

val GAINS by lazy {
    selectGainsBasedOnMode(
        Gains(
            kV = 1.87
        ),
        Gains(
            3.5,
            kV = 3.8
        )
    )
}
