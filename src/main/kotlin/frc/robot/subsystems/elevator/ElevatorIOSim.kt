package frc.robot.subsystems.elevator

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.ElevatorPorts

class ElevatorIOSim : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val limitSwitch = DigitalInput(ElevatorPorts.SENSOR_ID)
    private val motor = TalonFX(ElevatorPorts.MOTOR_ID)

    override fun setPosition(position: Double) {
        super.setPosition(position)
    }
    override fun SetPower(percentOutput: Double) {
        super.SetPower(percentOutput)
    override fun setPower(percentOutput: Double) {
        super.setPower(percentOutput)
    }
    override fun resat() {}
    override fun updateInputs() {
        super.updateInputs()
    }
}
