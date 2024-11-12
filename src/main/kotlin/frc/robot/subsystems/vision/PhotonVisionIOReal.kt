package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import java.util.Optional

class PhotonVisionIOReal(private val camera: PhotonCamera, private val robotToCam: Transform3d) : VisionIO {
    override val inputs = LoggedVisionInputs()
    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(
        VisionConstants.aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCam
    )

    override val name = camera.name

    init {
        camera.pipelineIndex = 0
    }

    override fun setPipeLine(pipeLineIndex: Int) {
        camera.pipelineIndex = pipeLineIndex
    }

    override fun updateInputs() {
        val unreadResults = camera.allUnreadResults

        lateinit var estimatedPose: Optional<EstimatedRobotPose>

        for (result in unreadResults) {
            if (!result.hasTargets()) {
                return
            }

            estimatedPose = estimator.update(result)

            if (estimatedPose.isEmpty) {
                return
            }

            val tags = result.targets

            inputs.distanceToTargets.clear()
            inputs.poseFieldOriented = estimatedPose.get().estimatedPose

            inputs.timestamp = estimatedPose.get().timestampSeconds

            for (tag in tags) {
                val distanceToTarget = tag.bestCameraToTarget.translation.norm
                inputs.distanceToTargets.add(distanceToTarget)
            }
        }
    }
}