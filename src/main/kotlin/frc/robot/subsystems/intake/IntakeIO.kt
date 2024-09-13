package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import org.team9432.annotation.Logged


interface IntakeIO {
    val inputs: LoggedIntakeInputs

    fun setCenterPower(power: Double) {}
    fun setRollerPower(power: Double){}
    fun setAngleMotorAngle(angle:Measure<Angle>){}
    fun setAngleMotorPow(power: Double){}
    fun reset(){}
    fun updateinputs(){}

    @Logged
    open class IntakeInputs {
        var spinMotorVoltage = 0.0
        var centerMotorVoltage = 0.0
        var angleMotorAngle: Measure<Angle> = Units.Degree.of(127.0)
        var angleMotorVoltage = 0.0
    }
}

