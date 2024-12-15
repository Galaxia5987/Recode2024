package frc.robot.subsystems.elevator

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.DigitalInput
import frc.robot.ElevatorPorts

class ElevatorIOSim : ElevatorIO {
    override val inputs = LoggedElevatorInputs()
    private val limitSwitch = DigitalInput(ElevatorPorts.SENSOR_ID)
    private val motor = TalonFX(ElevatorPorts.MOTOR_ID)

    override fun setHeight(position: Double) {
        super.setHeight(position)
    }
    override fun setPower(percentOutput: Double) {
        super.setPower(percentOutput)
    }
    override fun reset() {}
    override fun updateInputs() {
        super.updateInputs()
    }
}
