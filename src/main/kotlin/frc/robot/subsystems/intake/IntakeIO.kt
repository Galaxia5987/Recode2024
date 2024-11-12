package frc.robot.subsystems.intake

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import org.team9432.annotation.Logged

interface IntakeIO {

    val inputs: LoggedIntakeInput

    fun updateInput(){}

    fun setAngle(angle: Double){}

    fun resetAngle(){}

    fun setAnglePower(power: Double){}

    fun setsSpinMotorPower(power: Double){}

    fun setsCenterMotorPower(power: Double){}

    fun stopCenterMotor(){}

    fun stopSpinMotor(){}


    @Logged
    open class IntakeInput {
        var angle: Double = 0.0
        var angleMotorVoltage: Measure<Voltage> = Units.Volt.zero()
        var spinMotorVoltage: Measure<Voltage> = Units.Volt.zero()
        var centerMotorVoltage: Measure<Voltage> = Units.Volt.zero()
    }
}