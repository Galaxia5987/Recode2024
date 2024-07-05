package frc.robot.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.Ports

class IntakeIOReal : IntakeIO{
    val angleMotor = TalonFX(Ports.Intake.ANGLE_MOTOR_ID)
    val spinMotor = CANSparkMax(Ports.Intake.SPIN_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)
    val centerMotor = CANSparkMax(Ports.Intake.CENTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

}