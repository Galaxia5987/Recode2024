package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields

object VisionConstants {
    val aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
}