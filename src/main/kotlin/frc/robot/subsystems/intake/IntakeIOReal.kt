package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.Ports

class IntakeIOReal : IntakeIO{
    private val angleMotor = TalonFX(Ports.Intake.ANGLE_MOTOR_ID)
    private val spinMotor = CANSparkMax(Ports.Intake.SPIN_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val centerMotor = CANSparkMax(Ports.Intake.CENTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val positionControl = PositionVoltage(0.0)

    init {
        spinMotor.inverted = true
        spinMotor.setSmartCurrentLimit(40)
        spinMotor.idleMode = CANSparkBase.IdleMode.kCoast
        spinMotor.enableVoltageCompensation(12.0)

        centerMotor.inverted = true
        centerMotor.setSmartCurrentLimit(40)
        centerMotor.idleMode = CANSparkBase.IdleMode.kBrake
        centerMotor.enableVoltageCompensation(12.0)

//        angleMotor.configurator.apply(null) TODO: add configuration
    }

    override fun setSpinPower(power: Double) {
        spinMotor.set(power)
    }

    override fun setCenterPower(power: Double) {
        centerMotor.set(power)
    }

    override fun setAngle(angle: Rotation2d) {
//        angleMotor.setControl(positionControl)
    }

    override fun updateInputs() {
        inputs.angleMotorAngle = Units.Rotations.of(angleMotor.position.value)
        inputs.spinMotorVoltage = spinMotor.busVoltage
        inputs.centerMotorVoltage = centerMotor.busVoltage
    }
}