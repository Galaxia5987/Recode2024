package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.Ports

class IntakeIOReal : IntakeIO {

    override val inputs = LoggedIntakeInput()

    private val centerMotor: CANSparkMax =
        CANSparkMax(Ports.Intake.CENTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private val spinMotor: CANSparkMax = CANSparkMax(Ports.Intake.SPIN_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    private var angleMotor: TalonFX = TalonFX(Ports.Intake.ANGLE_MOTOR_ID)
    private val angleControl = PositionVoltage(0.0)


    init {
        angleMotor.configurator.apply(IntakeConstants.MOTOR_CONFIGURATION)
    }

    override fun updateInput() {
        inputs.angle = angleMotor.position.value * 2 * Math.PI
        inputs.spinMotorPower = spinMotor.get()
        inputs.angleMotorVoltage = angleMotor.supplyVoltage.value
        inputs.spinMotorPower = angleMotor.get()
    }

    override fun setAngle(angle: Double) {
        angleMotor.setControl(
            angleControl
                .withPosition(inputs.angle)
        )
    }

    override fun resetAngle() {
        angleMotor.setPosition(0.0)
    }

    override fun setAnglePower(power: Double) {
        angleMotor.set(power)
    }

    override fun setsSpinMotorPower(power: Double) {
        spinMotor.set(power)
    }

    override fun setsCenterMotorPower(power: Double) {
        centerMotor.set(power)
    }

}