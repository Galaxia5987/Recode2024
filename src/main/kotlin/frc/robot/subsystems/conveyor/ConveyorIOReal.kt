package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.hardware.TalonFX
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id
import frc.robot.Constants
import frc.robot.Ports

class ConveyorIOReal:ConveyorIO {
    override var inputs:LoggedConveyorInputs = LoggedConveyorInputs()
    private var spinMotor:TalonFX = TalonFX(Ports.Conveyor.MOTOR_ID)

    override fun updateInput() {
        inputs.spinMotorPower = spinMotor.get()
    }

    override fun setSpinPower(power: Double) {
        spinMotor.set(power)
    }
}