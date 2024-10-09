package frc.robot.subsystems.gripper

import org.team9432.annotation.Logged

interface GripperIO {
    val inputs: LoggedGripperInputs

    fun setPower(power: Double)
    fun updateInputs()

    @Logged
    open class GripperInputs {
        var spinMotorPower: Double = 0.0
        var hasNote = false
    }
}