package frc.robot.subsystems.shooter

import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.*
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

object ShooterConstants {
    const val GEAR_RATIO_TOP: Double = 1.0
    const val GEAR_RATIO_BOTTOM: Double = 1.0

    val TOP_ROLLER_TOLERANCE: Measure<Dimensionless> = Units.Percent.of(0.5)
    val BOTTOM_ROLLER_TOLERANCE: Measure<Dimensionless> = Units.Percent.of(0.5)
    val MOMENT_OF_INERTIA_TOP: Measure<Mult<Mult<Mass, Distance>, Distance>> =
        Units.Kilograms.mult(Units.Meters).mult(
            Units.Meters
        ).of(0.0008)
    val MOMENT_OF_INERTIA_BOTTOM: Measure<Mult<Mult<Mass, Distance>, Distance>> =
        Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.0008)

    val TOP_GAINS by lazy {
        selectGainsBasedOnMode(
            Gains(
                2.0,
                0.0,
                0.0
            ),
            Gains(
                0.4,
                0.0,
                0.0,
                0.0,
                0.1282,
                0.0
            )
        )
    }

    val BOTTOM_GAINS by lazy {
        selectGainsBasedOnMode(
            Gains(
                2.0,
                0.0,
                0.0
            ),
            Gains(
                0.3,
                0.0,
                0.0,
                0.0,
                0.1232,
                0.0
            )
        )
    }

    val STOP_POWER: Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()

    const val CURRENT_LIMIT_TOP = 40.0
    const val CURRENT_LIMIT_BOTTOM = 40.0

    val TOP_INVERSION = InvertedValue.CounterClockwise_Positive
    val BOTTOM_INVERSION = InvertedValue.Clockwise_Positive
}