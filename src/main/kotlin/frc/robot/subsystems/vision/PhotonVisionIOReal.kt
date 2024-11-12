package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

class PhotonVisionIOReal(private val camera: PhotonCamera, private val robotToCam: Transform3d) : VisionIO {
    override val inputs = LoggedVisionInputs()
    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(
        aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        robotToCam
    )

    override val name: String = camera.name

    init {
        camera.pipelineIndex = 0
    }

    override fun setPipeLine(pipeLineIndex: Int) {
        camera.pipelineIndex = pipeLineIndex
    }

    override fun updateInputs() {
        val unreadResults = camera.allUnreadResults

        for (result in unreadResults) {
            if (!result.hasTargets()) {
                return
            }

            val estimatedPose = estimator.update(result)

            if (estimatedPose.isEmpty) {
                continue
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