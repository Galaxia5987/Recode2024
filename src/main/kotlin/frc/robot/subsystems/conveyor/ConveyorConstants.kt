package frc.robot.subsystems.conveyor

import edu.wpi.first.units.*
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

object ConveyorConstants {
    const val GEAR_RATIO = 1.0

    val AT_SETPOINT_TOLERANCE: Measure<Dimensionless> = Units.Percent.of(0.5)

    val MOMENT_OF_INERTIA: Measure<Mult<Mult<Mass, Distance>, Distance>> =
        Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.000_05)

    val FEED_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(70.0)

    val GAINS by lazy {
        selectGainsBasedOnMode(
            Gains(
                kV= 1.87
            ),
            Gains(
                3.5,
                kV= 3.8
            )
        )
    }
}