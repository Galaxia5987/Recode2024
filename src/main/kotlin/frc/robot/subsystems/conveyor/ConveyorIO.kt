package frc.robot.subsystems.conveyor

import edu.wpi.first.units.*
import edu.wpi.first.units.measure.AngularVelocity
import org.team9432.annotation.Logged

interface ConveyorIO {
    val inputs: LoggedConveyorInputs

    fun updateInput() {}

    fun setSpinVelocity(vel: AngularVelocity) {}

    @Logged
    open class ConveyorInputs {
        var spinMotorVelocity: AngularVelocity = Units.RotationsPerSecond.zero()
    }
}