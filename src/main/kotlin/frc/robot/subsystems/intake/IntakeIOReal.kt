package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.IntakePorts

class IntakeIOReal : IntakeIO {
    override val inputs = LoggedIntakeInputs()
    
    private val angleMotor = TalonFX(IntakePorts.ANGLE_MOTOR_ID)
    private val spinMotor = TalonFX(IntakePorts.SPIN_MOTOR_ID)
    private val centerMotor = CANSparkMax(IntakePorts.CENTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val centerMotorConfigurator = SparkMaxConfig()

    private val positionControl = PositionVoltage(0.0)
    private val dutyCycle = DutyCycleOut(0.0)

    init {
        centerMotorConfigurator.apply {
            inverted(true)
            idleMode(SparkBaseConfig.IdleMode.kBrake)
            smartCurrentLimit(40)
            voltageCompensation(12.0)
        }
        centerMotor.configure(centerMotorConfigurator, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)

        val spinConfig = TalonFXConfiguration().apply {
            MotorOutput = MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }
        }

        spinMotor.configurator.apply(spinConfig)

        val angleConfig = TalonFXConfiguration().apply {
            MotorOutput = MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            Feedback = FeedbackConfigs().apply {
                RotorToSensorRatio = 1.0
                SensorToMechanismRatio = GEAR_RATIO
            }
            Slot0 = Slot0Configs().apply {
                kP = GAINS.kP
                kI = GAINS.kI
                kD = GAINS.kD

            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }
        }

        angleMotor.configurator.apply(angleConfig)

        angleMotor.setPosition(127.0 / 360.0)
    }

    override fun setSpinPower(power: Double) {
        spinMotor.set(power)
    }

    override fun setCenterPower(power: Double) {
        centerMotor.set(power)
    }

    override fun setAngle(angle: Measure<AngleUnit>) {
        angleMotor.setControl(positionControl.withPosition(angle.`in`(Units.Rotations)))
    }

    override fun setAnglePower(power: Double) {
        angleMotor.setControl(dutyCycle.withOutput(power))
    }

    override fun resetEncoder() {
        angleMotor.setPosition(0.0)
    }

    override fun setGains(kP: Double, kI: Double, kD: Double) {
        angleMotor.configurator.apply(
            Slot0Configs()
                .withKP(kP).withKI(kI).withKD(kD))
    }

    override fun updateInputs() {
        inputs.angleMotorAngle = angleMotor.position.value
        inputs.spinMotorVoltage = spinMotor.motorVoltage.value.`in`(Units.Volts)
        inputs.centerMotorVoltage = centerMotor.busVoltage
        inputs.angleMotorAppliedVoltage = angleMotor.motorVoltage.value.`in`(Units.Volts)
    }
}