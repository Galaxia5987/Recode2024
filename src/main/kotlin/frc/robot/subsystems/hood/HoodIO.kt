package frc.robot.subsystems.hood

import edu.wpi.first.units.*
import org.team9432.annotation.Logged

interface HoodIO {
    var inputs: LoggedInputHood

    fun updateInputs()

    fun setAngle(angle: Measure<Angle>)

    @Logged
    open class InputHood {
        var angle: MutableMeasure<Angle> = MutableMeasure.zero(Units.Rotations)
        var angleMotorVoltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var encoderPosition: MutableMeasure<Angle> = MutableMeasure.zero(Units.Rotations)
    }
}