package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.*
import frc.robot.Ports

class ShooterIOReal : ShooterIO {
    override val topRollerInputs = LoggedRollerInputs()
    override val bottomRollerInputs = LoggedRollerInputs()
    private val topMotor = TalonFX(Ports.Shooter.TOP_MOTOR_ID)
    private val bottomMotor = TalonFX(Ports.Shooter.BOTTOM_MOTOR_ID)
    private val topControl = VelocityVoltage(0.0)
    private val bottomControl = VelocityVoltage(0.0)

    init {
        topMotor.setNeutralMode(NeutralModeValue.Coast)
        bottomMotor.setNeutralMode(NeutralModeValue.Coast)

        val topMotorConfiguration = TalonFXConfiguration()
            .withFeedback(FeedbackConfigs().withSensorToMechanismRatio(ShooterConstants.GEAR_RATIO_TOP))
            .withSlot0(
                Slot0Configs()
                    .withKP(ShooterConstants.TOP_GAINS.kP)
                    .withKI(ShooterConstants.TOP_GAINS.kI)
                    .withKD(ShooterConstants.TOP_GAINS.kD)
                    .withKS(ShooterConstants.TOP_GAINS.kS)
                    .withKV(ShooterConstants.TOP_GAINS.kV)
                    .withKA(ShooterConstants.TOP_GAINS.kA)
            )
            .withMotorOutput(MotorOutputConfigs().withInverted(ShooterConstants.TOP_INVERSION)).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(2 * ShooterConstants.CURRENT_LIMIT_TOP)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT_TOP)


        val bottomMotorConfiguration = TalonFXConfiguration()
            .withFeedback(FeedbackConfigs().withSensorToMechanismRatio(ShooterConstants.GEAR_RATIO_BOTTOM))
            .withSlot0(
                Slot0Configs()
                    .withKP(ShooterConstants.BOTTOM_GAINS.kP)
                    .withKI(ShooterConstants.BOTTOM_GAINS.kI)
                    .withKD(ShooterConstants.BOTTOM_GAINS.kD)
                    .withKS(ShooterConstants.BOTTOM_GAINS.kS)
                    .withKV(ShooterConstants.BOTTOM_GAINS.kV)
                    .withKA(ShooterConstants.BOTTOM_GAINS.kA)
            )
            .withMotorOutput(MotorOutputConfigs().withInverted(ShooterConstants.BOTTOM_INVERSION)).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(2 * ShooterConstants.CURRENT_LIMIT_BOTTOM)
            .withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT_BOTTOM)

        topMotor.configurator.apply(topMotorConfiguration)
        bottomMotor.configurator.apply(bottomMotorConfiguration)
    }

    override fun setTopVelocity(velocity: Measure<Velocity<Angle>>) {
        topMotor.setControl(topControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun setBottomVelocity(velocity: Measure<Velocity<Angle>>) {
        bottomMotor.setControl(bottomControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun stop() {
        topMotor.stopMotor()
        bottomMotor.stopMotor()
    }

    override fun setTopPID(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {
        topMotor.configurator.apply(
            Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
        )
    }

    override fun setBottomPID(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {
        bottomMotor.configurator.apply(
            Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
        )
    }

    override fun updateInputs() {
        topRollerInputs.velocity.mut_replace(
            topMotor.velocity.value, Units.RotationsPerSecond
        )
        topRollerInputs.voltage.mut_replace(topMotor.motorVoltage.value, Units.Volts)

        bottomRollerInputs.velocity.mut_replace(
            bottomMotor.velocity.value, Units.RotationsPerSecond
        )
        bottomRollerInputs.voltage.mut_replace(bottomMotor.velocity.value, Units.Volts)
    }
}