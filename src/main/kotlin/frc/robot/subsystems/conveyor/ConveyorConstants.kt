package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.*
import frc.robot.Constants
import frc.robot.lib.webconstants.LoggedTunableNumber

object ConveyorConstants {
    const val GEAR_RATIO = 1.0

    val AT_SETPOINT_TOLERANCE: Measure<Dimensionless> = Units.Percent.of(0.5)

    val MOMENT_OF_INERTIA: Measure<Mult<Mult<Mass, Distance>, Distance>> =
        Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.000_05)

    val MOTOR_CONFIG: TalonFXConfiguration = TalonFXConfiguration()

    val FEED_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(70.0)

    val KP: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kP")
    val KI: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kI")
    val KD: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kD")
    val KS: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kS")
    val KV: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kV")
    val KA: LoggedTunableNumber = LoggedTunableNumber("Conveyor/kA")
    val PID_VALUES = arrayOf(KP, KI, KD, KS, KV, KA)

    init {
        when (Constants.CURRENT_MODE) {
            Constants.Mode.REAL, Constants.Mode.REPLAY -> {
                KP.initDefault(3.5)
                KI.initDefault(0.0)
                KD.initDefault(0.0)
                KS.initDefault(0.0)
                KV.initDefault(3.8)
                KA.initDefault(0.0)

                MOTOR_CONFIG
                    .withMotorOutput(
                        MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Coast)
                            .withInverted(InvertedValue.Clockwise_Positive)
                    ).withSlot0(
                        Slot0Configs()
                            .withKP(KP.get())
                            .withKI(KI.get())
                            .withKD(KD.get())
                            .withKS(KS.get())
                            .withKV(KV.get())
                            .withKA(KA.get())
                    ).withCurrentLimits(
                        CurrentLimitsConfigs()
                            .withStatorCurrentLimitEnable(true)
                            .withSupplyCurrentLimitEnable(true)
                            .withStatorCurrentLimit(80.0)
                            .withSupplyCurrentLimit(40.0)
                    ).withFeedback(
                        FeedbackConfigs()
                            .withSensorToMechanismRatio(GEAR_RATIO)
                    )
            }

            else -> {
                KP.initDefault(0.0)
                KI.initDefault(0.0)
                KD.initDefault(0.0)
                KS.initDefault(0.0)
                KV.initDefault(1.87)
                KA.initDefault(0.0)
            }
        }
    }
}