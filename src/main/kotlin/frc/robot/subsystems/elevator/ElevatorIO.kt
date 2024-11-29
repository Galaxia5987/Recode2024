package frc.robot.subsystems.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import org.team9432.annotation.Logged

interface ElevatorIO {
    val inputs: LoggedElevatorInputs

    fun setPosition(position: Double) {}

    fun SetPower(percentOutput: Double) {}

    fun isAtBottom(): Boolean {
        return false
    }

    fun updateInputs() {}

    @Logged
    open class ElevatorInputs {
        var carriageHeight: Distance = Units.Meters.of(0.0)
        var isAtBottom = false
    }
}
