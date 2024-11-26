package frc.robot.subsystems.gripper

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface GripperIO {
    val inputs: LoggedGripperInputs

    fun setPower(power: Double) {}
    fun stop() {}
    fun updateInputs() {}

    @Logged
    open class GripperInputs {
        var spinMotorVoltage: Voltage = Units.Volt.zero()
        var hasNote = false
    }
}