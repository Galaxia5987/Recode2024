package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.*
import frc.robot.lib.webconstants.LoggedTunableNumber

object ShooterConstants {
    const val GEAR_RATIO_TOP: Double = 1.0
    const val GEAR_RATIO_BOTTOM: Double = 1.0

    val TOP_ROLLER_TOLERANCE: Measure<Dimensionless> = Units.Percent.of(0.5)
    val BOTTOM_ROLLER_TOLERANCE: Measure<Dimensionless> = Units.Percent.of(0.5)
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

    private const val CURRENT_LIMIT_TOP = 40.0
    private const val CURRENT_LIMIT_BOTTOM = 40.0

    private val TOP_INVERSION = InvertedValue.CounterClockwise_Positive
    private val BOTTOM_INVERSION = InvertedValue.Clockwise_Positive

    init {
        topMotorConfiguration
            .withFeedback(FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO_TOP))
            .withSlot0(
                Slot0Configs()
                    .withKP(TOP_kP.get())
                    .withKI(TOP_kI.get())
                    .withKD(TOP_kD.get())
                    .withKS(TOP_kS.get())
                    .withKV(TOP_kV.get())
                    .withKA(TOP_kA.get())
            )
            .withMotorOutput(MotorOutputConfigs().withInverted(ShooterConstants.TOP_INVERSION)).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(2 * ShooterConstants.CURRENT_LIMIT_TOP)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT_TOP)

        bottomMotorConfiguration
            .withFeedback(FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO_BOTTOM))
            .withSlot0(
                Slot0Configs()
                    .withKP(BOTTOM_kP.get())
                    .withKI(BOTTOM_kI.get())
                    .withKD(BOTTOM_kD.get())
                    .withKS(BOTTOM_kS.get())
                    .withKV(BOTTOM_kV.get())
                    .withKA(BOTTOM_kA.get())
            )
            .withMotorOutput(MotorOutputConfigs().withInverted(ShooterConstants.BOTTOM_INVERSION)).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(2 * ShooterConstants.CURRENT_LIMIT_BOTTOM)
            .withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT_BOTTOM)
        TOP_kP.initDefault(5.0)
        BOTTOM_kP.initDefault(5.0)
    }
}