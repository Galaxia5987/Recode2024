package frc.robot.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.Ports

class IOReal:IntakeIO {
    private val motor= TalonFX(Ports.Intake.ANGLE_MOTOR_ID)
    private val spinMotor= CANSparkMax(Ports.Intake.SPIN_MOTOR_ID,CANSparkLowLevel.MotorType.kBrushless)
    private val centerMotor=CANSparkMax(Ports.Intake.CENTER_MOTOR_ID,CANSparkLowLevel.MotorType.kBrushless)

    init {
        spinMotor.restoreFactoryDefaults()
        spinMotor.setSmartCurrentLimit(40)
        spinMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        spinMotor.inverted=true
        spinMotor.burnFlash()

        centerMotor.restoreFactoryDefaults()
        centerMotor.setSmartCurrentLimit(40)
        centerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        centerMotor.inverted=true
        centerMotor.burnFlash()

        motor.configurator.apply(IntakeConstants.ANGLE_MOTOR_CONFIGURATION)
    }

    override fun setCenterPower(power: Double) {
        centerMotor.set(power)

    }

    override fun setAnglePower(power: Double) {
       spinMotor.set(power)
    }

    override fun setAngle(angle: Double) {
       motor.set(angle)
    }

    override fun reset() {

    }
}
