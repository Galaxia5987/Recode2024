package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d

data class VisionResult(
    var estimatedRobotPose: Pose3d = Pose3d(),
    var timestamp: Double = 0.0,
    var distanceToTargets: MutableList<Double> = ArrayList(),
) {
    override fun toString(): String {
        return """
            VisionResult(
                estimatedRobotPose: $estimatedRobotPose,
                timestamp: $timestamp,
                distanceToTargets: $distanceToTargets,
            )
        """.trimIndent()
    }
}