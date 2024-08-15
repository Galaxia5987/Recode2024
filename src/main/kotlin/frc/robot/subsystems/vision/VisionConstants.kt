package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d

object VisionConstants {
    val aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()

    var VISION_MEASUREMENT_MULTIPLIER = 1_000_000_000.0
    const val AUTO_VISION_MEASUREMENT_MULTIPLIER = 0.5;
    const val TELEOP_VISION_MEASUREMENT_MULTIPLIER = 0.05


    val SPEAKER_RIGHT_CAMERA_POSE = Transform3d(
        -0.065833,
        -0.04005,
        0.608178,
        Rotation3d(0.0, -Math.toRadians(25.0), Math.toRadians(180.0))
    )
    val SPEAKER_LEFT_CAMERA_POSE = Transform3d(
        -0.065833,
        0.03995,
        0.608178,
        Rotation3d(0.0, -Math.toRadians(25.0), Math.toRadians(180.0))
    )
    val INTAKE_APRILTAG_CAMERA_POSE = Transform3d(
        -0.012852, -0.0005, 0.635282, Rotation3d(0.0, -Math.toRadians(25.0), 0.0)
    )
    val DRIVER_CAMERA_POSE = Transform3d(0.0, 0.0, 0.53, Rotation3d(0.0, Math.toRadians(20.0), 0.0))
}