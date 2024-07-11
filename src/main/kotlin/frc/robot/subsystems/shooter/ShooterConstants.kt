package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.*
import frc.robot.lib.webconstants.LoggedTunableNumber

object ShooterConstants {
    const val GEAR_RATIO_TOP: Double = 1.0
    const val GEAR_RATIO_BOTTOM: Double = 1.0

    val TOP_ROLLER_TOLERANCE: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(1.0)
    val BOTTOM_ROLLER_TOLERANCE: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(1.0)
    val MOMENT_OF_INERTIA_TOP: MutableMeasure<Mult<Mult<Mass, Distance>, Distance>> =
        Units.Kilograms.mult(Units.Meters).mult(
            Units.Meters
        ).of(0.0008).mutableCopy()
    val MOMENT_OF_INERTIA_BOTTOM: MutableMeasure<Mult<Mult<Mass, Distance>, Distance>> =
        Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.0008).mutableCopy()

    val TOP_kP: LoggedTunableNumber = LoggedTunableNumber("Shooter/Top kP")
    val TOP_kI: LoggedTunableNumber = LoggedTunableNumber("Shooter/Top kI")
    val TOP_kD: LoggedTunableNumber = LoggedTunableNumber("Shooter/Top kD")
    val TOP_kS: LoggedTunableNumber = LoggedTunableNumber("Shooter/Top kS")
    val TOP_kV: LoggedTunableNumber = LoggedTunableNumber("Shooter/Top kV")
    val TOP_kA: LoggedTunableNumber = LoggedTunableNumber("Shooter/Top kA")

    val BOTTOM_kP: LoggedTunableNumber = LoggedTunableNumber("Shooter/Bottom kP")
    val BOTTOM_kI: LoggedTunableNumber = LoggedTunableNumber("Shooter/Bottom kI")
    val BOTTOM_kD: LoggedTunableNumber = LoggedTunableNumber("Shooter/Bottom kD")
    val BOTTOM_kS: LoggedTunableNumber = LoggedTunableNumber("Shooter/Bottom kS")
    val BOTTOM_kV: LoggedTunableNumber = LoggedTunableNumber("Shooter/Bottom kV")
    val BOTTOM_kA: LoggedTunableNumber = LoggedTunableNumber("Shooter/Bottom kA")

    val topMotorConfiguration = TalonFXConfiguration()
    val bottomMotorConfiguration = TalonFXConfiguration()

    val STOP_POWER = Units.RotationsPerSecond.zero().mutableCopy()

}