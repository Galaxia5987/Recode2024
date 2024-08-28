package frc.robot.subsystems.conveyor

import edu.wpi.first.units.*
import frc.robot.Constants
import frc.robot.lib.Gains
import frc.robot.lib.LoggedTunableNumber

object ConveyorConstants {
    const val GEAR_RATIO = 1.0

    val AT_SETPOINT_TOLERANCE: Measure<Dimensionless> = Units.Percent.of(0.5)

    val MOMENT_OF_INERTIA: Measure<Mult<Mult<Mass, Distance>, Distance>> =
        Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.000_05)

    val FEED_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(70.0)

    val GAINS by lazy {
        if (Constants.CURRENT_MODE == Constants.Mode.SIM) Gains(
            0.0,
            0.0,
            0.0,
            0.0,
            1.87,
            0.0
        ) else Gains(
            3.5,
            0.0,
            0.0,
            0.0,
            3.8,
            0.0
        )
    }
}