package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.*
import frc.robot.lib.webconstants.LoggedTunableNumber

object ConveyorConstants {
    const val GEAR_RATIO = 1.0

    val AT_SETPOINT_TOLERANCE: Measure<Dimensionless> = Units.Percent.of(0.5)

    val MOMENT_OF_INERTIA : Measure<Mult<Mult<Mass, Distance>, Distance>> = Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.000_05)

    val MOTOR_CONFIG: TalonFXConfiguration = TalonFXConfiguration()

    val FEED_VELOCITY: MutableMeasure<Velocity<Angle>> = Units.RotationsPerSecond.of(70.0).mutableCopy()

    val KP: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kP")
    val KI: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kI")
    val KD: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kD")
    val KS: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kS")
    val KV: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kV")
    val KA: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kA")
}