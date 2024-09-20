package frc.robot.subsystems.gripper

import org.team9432.annotation.Logged

interface gripperIO {
    val inputs: LoggedgripperInputs
    fun setPower(power: Double)
    fun stop()

    @Logged
    open class gripperInputs {
        var note = false
        var voltage = 0.0

    }
}