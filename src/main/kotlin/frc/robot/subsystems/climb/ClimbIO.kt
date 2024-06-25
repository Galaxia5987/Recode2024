package frc.robot.subsystems.climb

interface ClimbIO {

    fun setPower(power:Double)

    fun openStopper()

    fun closeStopper()

    fun disableStopper()

    fun updateInputs()
}