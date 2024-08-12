package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.lib.webconstants.LoggedTunableNumber
import org.team9432.annotation.Logged

interface IntakeIO {
    val inputs: LoggedIntakeInputs

    fun setSpinPower(power: Double) {}

    fun setCenterPower(power: Double) {}

    fun setAngle(angle: Measure<Angle>) {}

    fun setAnglePower(power: Double) {}

    fun resetEncoder() {}

    fun updateInputs() {}

    fun updatePID() {}

    fun hasPIDChanged(PIDValues: Array<LoggedTunableNumber>): Boolean{
        var hasChanged = false
        for (value in PIDValues){
            if (value.hasChanged()) hasChanged = true
        }
        return hasChanged
    }

    @Logged
    open class IntakeInputs {
        var spinMotorVoltage = 0.0
        var centerMotorVoltage = 0.0
        var angleMotorAngle: Measure<Angle> = Units.Degree.of(120.0)
        var angleMotorAppliedVoltage = 0.0
    }
}
