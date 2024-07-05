package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.*
import frc.robot.lib.webconstants.LoggedTunableNumber

object HoodConstants {
    const val GEAR_RATIO : Double = 3.0 * (36.0 / 18.0) * (158.0 / 18.0)
    const val MAX_TOLERANCE_DEG = 0.75 // Degrees
    const val ENCODER_TICKS_PER_REVOLUTION = 4096

    val MOMENT_OF_INERTIA : Measure<Mult<Mult<Mass, Distance>, Distance>> = Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.0003);
    val MAX_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(1.0)
    val MAX_ACCELERATION: Measure<Velocity<Velocity<Angle>>> = Units.RotationsPerSecond.per(Units.Second).of(4.0)

    val kP = LoggedTunableNumber("Hood/kP")
    val kI = LoggedTunableNumber("Hood/kI")
    val kD = LoggedTunableNumber("Hood/kD")
    val kS = LoggedTunableNumber("Hood/kS")
    val kV = LoggedTunableNumber("Hood/kV")
    val kA = LoggedTunableNumber("Hood/kA")
    val kG = LoggedTunableNumber("Hood/kG")

    val MOTOR_CONFIGURATION = TalonFXConfiguration()
}