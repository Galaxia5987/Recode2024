package frc.robot.subsystems.hood

import edu.wpi.first.units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface HoodIO {
    var inputs: LoggedInputHood

    fun updateInputs() {}

    fun setAngle(angle: Angle) {}

    @Logged
    open class InputHood {
        var angle: Angle = Units.Rotations.zero()
        var angleMotorVoltage: Voltage = Units.Volt.zero()
        var encoderPosition: Angle = Units.Rotations.zero()
    }
}