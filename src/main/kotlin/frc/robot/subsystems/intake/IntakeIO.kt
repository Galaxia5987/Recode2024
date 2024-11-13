package frc.robot.subsystems.intake

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import org.team9432.annotation.Logged

interface IntakeIO {
    val inputs: LoggedIntakeInputs

    fun setSpinPower(power: Double) {}

    fun setCenterPower(power: Double) {}

    fun setAngle(angle: Angle) {}

    fun setAnglePower(power: Double) {}

    fun resetEncoder() {}

    fun setGains(kP: Double, kI: Double, kD: Double) {}

    fun updateInputs() {}

    @Logged
    open class IntakeInputs {
        var spinMotorVoltage = 0.0
        var centerMotorVoltage = 0.0
        var angleMotorAngle: Angle = Units.Degree.of(120.0)
        var angleMotorAppliedVoltage = 0.0
    }
}
