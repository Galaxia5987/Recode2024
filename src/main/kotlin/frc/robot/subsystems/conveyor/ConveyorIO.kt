package frc.robot.subsystems.conveyor

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import org.team9432.annotation.Logged

interface ConveyorIO {
    var inputs:LoggedConveyorInputs

    fun updateInput()

    fun setSpinVelocity(vel:Measure<Velocity<Angle>>)

    @Logged
    open class ConveyorInputs{
        var spinMotorVelocity:MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
    }
}