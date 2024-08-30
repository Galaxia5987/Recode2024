package frc.robot.subsystems.climb

interface ClimbIO {
    val inputs: LoggedClimbInputs

    fun updateInput()

    fun setPower(power:Double)

    fun lockClimb()

    fun unlockClimb()

    fun disableLockMotor()

    open class ClimbInputs {
        var climbMotorVoltage = 0.0
        var lockMotorCurrent = 0.0

    }
}