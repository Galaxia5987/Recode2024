package frc.robot.subsystems.hood

import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.*
import frc.robot.lib.Gains
import frc.robot.lib.LoggedTunableNumber
import frc.robot.lib.selectGainsBasedOnMode

object HoodConstants {
    const val GEAR_RATIO: Double = 3.0 * (36.0 / 18.0) * (158.0 / 18.0)
    const val ENCODER_TICKS_PER_REVOLUTION = 4096.0
    const val SIMULATION_LENGTH = 3.0
    val MAX_TOLERANCE: Measure<Dimensionless> = Units.Percent.of(0.03)

    val MOMENT_OF_INERTIA: Measure<Mult<Mult<Mass, Distance>, Distance>> =
        Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.0003);
    val MAX_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(1.0)
    val MAX_ACCELERATION: Measure<Velocity<Velocity<Angle>>> = Units.RotationsPerSecond.per(Units.Second).of(4.0)
    val RESTING_ANGLE: Measure<Angle> = Units.Degrees.of(100.0)
    val HOOD_LENGTH: Measure<Distance> = Units.Meters.of(0.4)

    val MECHANISM_2D_POSE = Translation2d(1.0, 1.0)
    val ROOT_POSITION = Translation3d(-0.27, 0.0, 0.225)
    val SIMULATION_OFFSET: Measure<Angle> = Units.Degrees.of(-54.0)

    const val CURRENT_LIMIT = 40.0
    val INVERTED_VALUE = InvertedValue.CounterClockwise_Positive


    val ABSOLUTE_ENCODER_OFFSET = LoggedTunableNumber("Hood/EncoderOffset", (3.9 - 33.48) / 360.0)

    val GAINS by lazy {
        selectGainsBasedOnMode(
            Gains(
                kP = 1700.0, kD = 335.0, kG =  5.5
            ), Gains(
                20.0
            )
        )
    }
}