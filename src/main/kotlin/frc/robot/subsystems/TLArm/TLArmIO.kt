package frc.robot.subsystems.TLArm

import org.team9432.annotation.Logged

interface TLArmIO {           //TL for telescopic
    var inputs:LoggedTLArmInputs
    fun updateInput()
    fun setPosition(setPoint:Double)

    @Logged
    open class TLArmInputs {
        var currentPose = 0.0
    }
}