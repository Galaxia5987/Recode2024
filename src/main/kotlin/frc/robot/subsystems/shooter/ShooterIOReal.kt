package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.*
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

        val topMotorConfig = TalonFXConfiguration().apply {
            Feedback = FeedbackConfigs().apply {
                SensorToMechanismRatio = ShooterConstants.GEAR_RATIO_TOP
            }
            Slot0 = Slot0Configs().apply {
                kP = ShooterConstants.TOP_GAINS.kP
                kI = ShooterConstants.TOP_GAINS.kI
                kD = ShooterConstants.TOP_GAINS.kD
                kS = ShooterConstants.TOP_GAINS.kS
                kV = ShooterConstants.TOP_GAINS.kV
                kA = ShooterConstants.TOP_GAINS.kA
            }
            MotorOutput = MotorOutputConfigs().apply {
                Inverted = ShooterConstants.TOP_INVERSION
            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 2 * ShooterConstants.CURRENT_LIMIT_TOP
                SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT_TOP
            }
        }

        val bottomMotorConfig = TalonFXConfiguration().apply {
            Feedback = FeedbackConfigs().apply {
                SensorToMechanismRatio = ShooterConstants.GEAR_RATIO_BOTTOM
            }
            Slot0 = Slot0Configs().apply {
                kP = ShooterConstants.BOTTOM_GAINS.kP
                kI = ShooterConstants.BOTTOM_GAINS.kI
                kD = ShooterConstants.BOTTOM_GAINS.kD
                kS = ShooterConstants.BOTTOM_GAINS.kS
                kV = ShooterConstants.BOTTOM_GAINS.kV
                kA = ShooterConstants.BOTTOM_GAINS.kA
            }
            MotorOutput = MotorOutputConfigs().apply {
                Inverted = ShooterConstants.BOTTOM_INVERSION
            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 2 * ShooterConstants.CURRENT_LIMIT_BOTTOM
                SupplyCurrentLimit = ShooterConstants.CURRENT_LIMIT_BOTTOM
            }
        }

        topMotor.configurator.apply(topMotorConfig)
        bottomMotor.configurator.apply(bottomMotorConfig)
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

    override fun setTopGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {
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

    override fun setBottomGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {
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