package frc.robot.subsystems.hood

import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import org.team9432.annotation.Logged

interface HoodIO {
    var inputs:LoggedInputHood

    fun updateInput()

    fun setAngle(angle:Double)

    @Logged
    open class InputHood{
        var angle:MutableMeasure<Angle> =
        var angleMotorVoltage = 0.0
        var encoderPosition = 0.0
    }
}