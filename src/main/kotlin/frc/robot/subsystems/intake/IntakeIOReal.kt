package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.Ports

class IntakeIOReal : IntakeIO {
    override val inputs = LoggedIntakeInputs()
    private val angleMotor = TalonFX(Ports.Intake.ANGLE_MOTOR_ID)
    private val spinMotor = CANSparkMax(Ports.Intake.SPIN_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val centerMotor = CANSparkMax(Ports.Intake.CENTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val positionControl = PositionVoltage(0.0)
    private val dutyCycle = DutyCycleOut(0.0)

    init {
        spinMotor.restoreFactoryDefaults()
        spinMotor.inverted = true
        spinMotor.setSmartCurrentLimit(40)
        spinMotor.idleMode = CANSparkBase.IdleMode.kCoast
        spinMotor.enableVoltageCompensation(12.0)
        spinMotor.burnFlash()

        centerMotor.restoreFactoryDefaults()
        centerMotor.inverted = true
        centerMotor.setSmartCurrentLimit(40)
        centerMotor.idleMode = CANSparkBase.IdleMode.kBrake
        centerMotor.enableVoltageCompensation(12.0)
        centerMotor.burnFlash()

        val config = TalonFXConfiguration().withMotorOutput(
            MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive)
        ).withFeedback(
            FeedbackConfigs().withRotorToSensorRatio(1.0).withSensorToMechanismRatio(IntakeConstants.GEAR_RATIO)
        ).withSlot0(
            Slot0Configs().withKP(IntakeConstants.Gains.kP).withKI(IntakeConstants.Gains.kI)
                .withKD(IntakeConstants.Gains.kD)
        ).CurrentLimits.withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80.0).withSupplyCurrentLimit(40.0)

        angleMotor.configurator.apply(config)

        angleMotor.setPosition(120.0 / 360.0)
    }

    override fun setSpinPower(power: Double) {
        spinMotor.set(power)
    }

    override fun setCenterPower(power: Double) {
        centerMotor.set(power)
    }

    override fun setAngle(angle: Measure<Angle>) {
        angleMotor.setControl(positionControl.withPosition(angle.`in`(Units.Rotations)))
    }

    override fun setAnglePower(power: Double) {
        angleMotor.setControl(dutyCycle.withOutput(power))
    }

    override fun resetEncoder() {
        angleMotor.setPosition(0.0)
    }

    override fun setPID(kP: Double, kI: Double, kD: Double) {

    }

    override fun updateInputs() {
        inputs.angleMotorAngle = Units.Rotations.of(angleMotor.position.value)
        inputs.spinMotorVoltage = spinMotor.busVoltage
        inputs.centerMotorVoltage = centerMotor.busVoltage
        inputs.angleMotorAppliedVoltage = angleMotor.motorVoltage.value
    }
}