package frc.robot.subsystems.climb

import org.team9432.annotation.Logged

interface ClimbIO {
    val inputs: LoggedClimbInputs

    fun updateInput()

    fun setPower(power: Double)

    fun lockClimb()

    fun unlockClimb()

    fun disableLockMotor()

    @Logged
    open class ClimbInputs {
        var climbMotorVoltage = 0.0
        var lockMotorCurrent = 0.0

    }
}