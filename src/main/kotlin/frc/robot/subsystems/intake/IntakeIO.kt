package frc.robot.subsystems.intake

import org.team9432.annotation.Logged

interface IntakeIO {

    val inputs: LoggedIntakeInput

    fun updateInput()

    fun setAngle(angle: Double)

    fun resetAngle()

    fun setAnglePower(power: Double)

    fun setsSpinMotorPower(power: Double)

    fun setsCenterMotorPower(power: Double)

    @Logged
    open class IntakeInput {
        var angle: Double = 0.0
        var angleMotorVoltage: Double = 0.0
        var spinMotorPower: Double = 0.0
        var centerMotorPower: Double = 0.0
    }
}