package frc.robot.subsystems.climb

import edu.wpi.first.units.Units
import org.team9432.annotation.Logged

interface ClimbIO {
    val inputs: LoggedClimbInputs

    fun setPower(power: Double) {}

    fun openStopper() {}

    fun closeStopper() {}

    fun disableStopper() {}

    fun updateInputs() {}

    @Logged
    open class ClimbInputs {
        var stopperAppliedVoltage = 0.0
        var stopperCurrent = 0.0
        var mainMotorAppliedVoltage = Units.Volts.zero()
    }
}
