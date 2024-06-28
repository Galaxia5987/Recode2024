package frc.robot.subsystems.climb

interface ClimbIO {
    var isStopperStuck: Boolean

    fun setPower(power:Double)

    fun openStopper()

    fun closeStopper()

    fun disableStopper()

    fun updateInputs()
}