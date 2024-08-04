package frc.robot.subsystems.climb

import edu.wpi.first.units.Current
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
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
        var stopperAppliedVoltage: MutableMeasure<Voltage> = Units.Volts.of(0.0).mutableCopy()
        var stopperCurrent: MutableMeasure<Current> = Units.Amps.of(0.0).mutableCopy()
        var mainMotorAppliedVoltage: MutableMeasure<Voltage> = Units.Volts.of(0.0).mutableCopy()
        var isStopperStuck = false
    }
}