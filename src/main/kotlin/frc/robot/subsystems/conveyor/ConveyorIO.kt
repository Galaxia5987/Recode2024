package frc.robot.subsystems.conveyor

import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import frc.robot.lib.webconstants.LoggedTunableNumber
import org.team9432.annotation.Logged

interface ConveyorIO {
    val inputs: LoggedConveyorInputs

    fun setVelocity(velocity: MutableMeasure<Velocity<Angle>>) {}

    fun updateInputs() {}

    fun stop() {}

    fun updatePID() {}

    fun hasPIDChanged(PIDValues: Array<LoggedTunableNumber>): Boolean {
        var hasChanged = false
        for (value in PIDValues) {
            if (value.hasChanged()) hasChanged = true
        }
        return hasChanged
    }

    @Logged
    open class ConveyorInputs {
        var velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
    }
}