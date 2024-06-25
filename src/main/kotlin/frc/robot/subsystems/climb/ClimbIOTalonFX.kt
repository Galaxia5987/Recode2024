package frc.robot.subsystems.climb

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.Servo
import frc.robot.Ports
import frc.robot.main

class ClimbIOTalonFX : ClimbIO {
    private val mainMotor = TalonFX(Ports.Climb.mainMotorID)
    private val auxMotor = TalonFX(Ports.Climb.auxMotorID)
    private val servo = Servo(Ports.Climb.servoID)

    init {
        mainMotor.configurator.apply()
    }

    override fun setPower(power: Double) {
        TODO("Not yet implemented")
    }

    override fun openStopper() {
        TODO("Not yet implemented")
    }

    override fun closeStopper() {
        TODO("Not yet implemented")
    }

    override fun disableStopper() {
        TODO("Not yet implemented")
    }

    override fun updateInputs() {
        TODO("Not yet implemented")
    }
}