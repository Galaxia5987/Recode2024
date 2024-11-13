package frc.robot.subsystems.gripper

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface GripperIO {
    val inputs: LoggedGripperInputs

    fun setRollerMotorPower(power: Double) {}

    fun updateInputs() {}

    @Logged
    open class GripperInputs {
        var rollerMotorVoltage: Voltage = Units.Volts.of(0.0).mutableCopy()
        var hasNote = false
    }
}