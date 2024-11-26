package frc.robot.subsystems.climb

import edu.wpi.first.units.CurrentUnit
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ClimbIO {
    val inputs: LoggedClimbInputs

    fun updateInput() {}

    fun setPower(power: Double) {}

    fun lockClimb() {}

    fun unlockClimb() {}

    fun disableLockMotor() {}

    @Logged
    open class ClimbInputs {
        var climbMotorVoltage: Voltage = Units.Volt.zero()
        var lockMotorCurrent: Current = Units.Amps.zero()

    }
}