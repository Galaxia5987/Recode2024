package frc.robot.subsystems.conveyor

import org.team9432.annotation.Logged

interface ConveyorIO {
    var inputs:LoggedConveyorInputs

    fun updateInput()

    fun setSpinPower(power:Double)

    @Logged
    open class ConveyorInputs{
        var spinMotorPower:Double = 0.0
    }
}