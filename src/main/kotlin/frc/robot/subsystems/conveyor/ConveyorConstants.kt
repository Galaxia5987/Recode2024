package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.*
import frc.robot.lib.webconstants.LoggedTunableNumber

object ConveyorConstants {
    const val GEAR_RATIO = 1.0
    const val TOLERANCE = 1.0

    val MOMENT_OF_INERTIA : Measure<Mult<Mult<Mass, Distance>, Distance>> = Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.000_05)

    val MOTOR_CONFIG: TalonFXConfiguration = TalonFXConfiguration()

    val FEED_VELOCITY: MutableMeasure<Velocity<Angle>> = Units.RotationsPerSecond.of(70.0).mutableCopy()

    var KP: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kP")
    var KI: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kI")
    var KD: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kD")
    var KS: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kS")
    var KV: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kV")
    var KA: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kA")
}