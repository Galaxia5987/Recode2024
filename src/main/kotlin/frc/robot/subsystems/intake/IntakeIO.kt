package frc.robot.subsystems.intake
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import org.team9432.annotation.Logged

 interface IntakeIO {
    val inputs: LoggedIntakeInputs
    fun setSpinPower(power: Double) {}
    fun setCenterPower(power: Double) {}
    fun setAnglePower(power: Double) {}
    fun setAngle(angle: Measure<Angle>) {}
    fun reset() {}
    fun updateInputs() {}

    @Logged
    open class IntakeInputs {
        var spinmMotorVoltage = 0.0
        var centerMotorVoltage = 0.0
        var angleMotorAngle: Measure<Angle> = Units.Degree.zero()
    }
}