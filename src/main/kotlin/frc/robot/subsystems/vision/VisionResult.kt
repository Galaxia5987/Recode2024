package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d

data class VisionResult(
    var estimatedRobotPose: Pose3d,
    var timestamp: Double,
    var distanceToTargets: MutableList<Double> = ArrayList(),
    var poseAmbiguities: MutableList<Double> = ArrayList()
)