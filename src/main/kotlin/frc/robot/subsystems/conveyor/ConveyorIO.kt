package frc.robot.subsystems.conveyor

import edu.wpi.first.units.*
import org.team9432.annotation.Logged

interface ConveyorIO {
    var inputs: LoggedConveyorInputs

    fun updateInput()

    fun setSpinVelocity(vel: Measure<Velocity<Angle>>)

    @Logged
    open class ConveyorInputs {
        val spinMotorVelocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
    }
}