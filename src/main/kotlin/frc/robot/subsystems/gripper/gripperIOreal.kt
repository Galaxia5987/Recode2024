package frc.robot.subsystems.gripper

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.Ports

class gripperIOreal():gripperIO {
    override val inputs: LoggedgripperInputs = LoggedgripperInputs()
    private val rollerMotor = CANSparkMax(Ports.Gripper.ROLLER_ID, CANSparkLowLevel.MotorType.kBrushless)

}