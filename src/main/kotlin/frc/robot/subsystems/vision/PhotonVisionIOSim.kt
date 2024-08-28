package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import frc.robot.subsystems.swerve.SwerveDrive
import org.photonvision.PhotonPoseEstimator
import org.photonvision.estimation.TargetModel
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.VisionTargetSim
import org.photonvision.targeting.PhotonPipelineResult

class PhotonVisionIOSim(private val simCamera: PhotonCameraSim, private val robotToCam: Transform3d) : VisionIO {
    override val inputs = LoggedVisionInputs()
    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(
        VisionConstants.aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        simCamera.camera,
        robotToCam
    )


    init {
        VisionSim.system.addAprilTags(VisionConstants.aprilTagFieldLayout)
        VisionSim.system.addCamera(simCamera, robotToCam)
    }

    override fun getLatestResult(): VisionResult =
        VisionResult(inputs.poseFieldOriented, inputs.timestamp, inputs.distanceToTargets, inputs.poseAmbiguities)

    private fun pose2dToPose3d(pose: Pose2d): Pose3d = Pose3d(
        pose.x, pose.y, 0.0, Rotation3d(0.0, 0.0, pose.rotation.radians)
    )

    override fun updateInputs() {
        val botPose = SwerveDrive.getInstance().estimator.estimatedPosition
        val botPose3d = pose2dToPose3d(botPose)
        val latestResult =
            simCamera.process(
                0.0,
                botPose3d + robotToCam.inverse(),
                VisionConstants.aprilTagFieldLayout.tags.map { a ->
                    VisionTargetSim(
                        a.pose,
                        TargetModel.kAprilTag36h11,
                        a.ID
                    )
                }
            )
        simCamera.submitProcessedFrame(latestResult)

        if (!latestResult.hasTargets()) {
            return
        }

        val estimatedPose = estimator.update(latestResult)

        inputs.poseFieldOriented = estimatedPose.get().estimatedPose
        inputs.timestamp = estimatedPose.get().timestampSeconds

        val tags = latestResult.targets

        for (tag in tags) {
            inputs.distanceToTargets.add(tag.bestCameraToTarget.translation.norm)
            inputs.tagAreas.add(tag.area)
            inputs.poseAmbiguities.add(tag.poseAmbiguity)
        }

    }
}