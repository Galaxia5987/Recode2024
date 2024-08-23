package frc.robot.lib.extensions

import edu.wpi.first.math.kinematics.ChassisSpeeds
import kotlin.math.hypot

object ChassisSpeedsExtensions {
fun ChassisSpeeds.getSpeed() = hypot(vxMetersPerSecond, vyMetersPerSecond)
}