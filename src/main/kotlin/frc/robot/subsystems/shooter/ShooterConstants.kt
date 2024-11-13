package frc.robot.subsystems.shooter

import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Dimensionless
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

const val GEAR_RATIO_TOP: Double = 1.0
const val GEAR_RATIO_BOTTOM: Double = 1.0

val TOP_ROLLER_TOLERANCE: Dimensionless = Units.Percent.of(0.03)
val BOTTOM_ROLLER_TOLERANCE: Dimensionless = Units.Percent.of(0.03)
val MOMENT_OF_INERTIA_TOP: MomentOfInertia = Units.KilogramSquareMeters.of(.0008)
val MOMENT_OF_INERTIA_BOTTOM: MomentOfInertia = Units.KilogramSquareMeters.of(.0008)

val TOP_GAINS by lazy {
    selectGainsBasedOnMode(
        Gains(
            0.4,
            kV = 0.1282
        ),
        Gains(
            2.0,
            0.0,
            0.0
        )
    )
}

val BOTTOM_GAINS by lazy {
    selectGainsBasedOnMode(
        Gains(
            0.3,
            kV = 0.1232
        ),
        Gains(
            2.0,
            0.0,
            0.0
        )
    )
}

val STOP_POWER: AngularVelocity = Units.RotationsPerSecond.zero()

const val CURRENT_LIMIT_TOP = 40.0
const val CURRENT_LIMIT_BOTTOM = 40.0

val TOP_INVERSION = InvertedValue.CounterClockwise_Positive
val BOTTOM_INVERSION = InvertedValue.Clockwise_Positive
