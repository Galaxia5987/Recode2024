package frc.robot.lib.extensions

import edu.wpi.first.math.kinematics.ChassisSpeeds
import kotlin.math.pow
import kotlin.math.sqrt

object ChassisSpeedsExtensions {
    fun ChassisSpeeds.getVelocityMagnitude(): Double{
        return sqrt(vxMetersPerSecond.pow(2)+vyMetersPerSecond.pow(2))
    }
}