package frc.robot.subsystems.conveyor

import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import org.team9432.annotation.Logged

interface ConveyorIO {
    val inputs: LoggedConveyorInputs
        get() = LoggedConveyorInputs()

    fun setVelocity(velocity: MutableMeasure<Velocity<Angle>>) {}

    fun updateInputs() {}

    fun stop() {}

    @Logged
    open class ConveyorInputs {
        var velocitySetPoint: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
        var velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
    }
}