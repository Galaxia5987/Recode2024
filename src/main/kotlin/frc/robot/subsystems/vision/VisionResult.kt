package frc.robot.subsystems.vision

import org.photonvision.EstimatedRobotPose

data class VisionResult(
    var estimatedRobotPose: EstimatedRobotPose,
    var useForEstimation: Boolean
)