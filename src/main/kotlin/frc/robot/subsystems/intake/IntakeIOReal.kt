package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Ports

class IntakeIOReal : IntakeIO{
    override val inputs = LoggedIntakeInputs()
    private val angleMotor = TalonFX(Ports.Intake.ANGLE_MOTOR_ID)
    private val spinMotor = CANSparkMax(Ports.Intake.SPIN_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val centerMotor = CANSparkMax(Ports.Intake.CENTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val positionControl = PositionVoltage(0.0)
    private val dutyCycle = DutyCycleOut(0.0)

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

    override fun setAngle(angle: Measure<Angle>) {
        angleMotor.setControl(positionControl.withPosition(angle.`in`(Units.Degrees)))
    }

    override fun setAnglePower(power: Double) {
        angleMotor.setControl(dutyCycle.withOutput(power))
    }

    override fun resetEncoder() {
        angleMotor.setPosition(0.0)
    }

    override fun updateInputs() {
        inputs.angleMotorAngle = Units.Degree.of(angleMotor.position.value)
        inputs.spinMotorVoltage = spinMotor.busVoltage
        inputs.centerMotorVoltage = centerMotor.busVoltage

    }
}