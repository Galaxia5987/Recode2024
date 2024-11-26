package frc.robot.subsystems.intake

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface IntakeIO {

    val inputs: LoggedIntakeInput

    fun updateInput() {}

    fun setAngle(angle: Angle) {}

    fun resetAngle() {}

    fun setAnglePower(power: Double) {}

    fun setsSpinMotorPower(power: Double) {}

    fun setsCenterMotorPower(power: Double) {}

    fun stopCenterMotor() {}

    fun stopSpinMotor() {}


    @Logged
    open class IntakeInput {
        var angle: Angle = Units.Degree.zero()
        var angleMotorVoltage: Voltage = Units.Volt.zero()
        var spinMotorVoltage: Voltage = Units.Volt.zero()
        var centerMotorVoltage: Voltage = Units.Volt.zero()
    }
}