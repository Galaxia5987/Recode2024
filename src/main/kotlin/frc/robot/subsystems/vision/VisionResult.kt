package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import org.photonvision.EstimatedRobotPose

data class VisionResult(
    var estimatedRobotPose: Pose3d,
    var timestamp: Double,
    var ambiguity: Double
)