package frc.robot.subsystems.hood

import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import org.team9432.annotation.Logged

interface HoodIO {
    var inputs:LoggedInputHood

    fun updateInput()

    fun setAngle(angle:Double)

    @Logged
    open class InputHood{
        var angle:MutableMeasure<Angle> = MutableMeasure.zero(Units.Rotations)
        var angleSetPoint:MutableMeasure<Angle> = MutableMeasure.zero(Units.Rotations)
        var angleMotorVoltage:MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var encoderPosition:MutableMeasure<Angle> = MutableMeasure.zero(Units.Rotations)
    }
}