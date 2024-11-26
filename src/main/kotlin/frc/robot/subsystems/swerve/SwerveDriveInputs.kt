package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team9432.annotation.Logged

@Logged
open class SwerveDriveInputs {
    // x, y, omega
    var desiredSpeeds = ChassisSpeeds()

    var acceleration = 0.0F

    var rawYaw = Rotation2d()
    var yaw = Rotation2d()
    var gyroOffset = Rotation2d()
}
