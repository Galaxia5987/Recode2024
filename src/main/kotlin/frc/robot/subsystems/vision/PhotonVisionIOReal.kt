package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.units.Units
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

class PhotonVisionIOReal(private val camera: PhotonCamera, private val robotToCam: Transform3d) : VisionIO {
    override val inputs = LoggedVisionInputs()
    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(
        VisionConstants.aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        robotToCam
    )

    init {
        camera.pipelineIndex = 0
    }

    override fun setPipeLine(pipeLineIndex: Int) {
        camera.pipelineIndex = pipeLineIndex
    }

    override fun getLatestResult(): VisionResult =
        VisionResult(inputs.poseFieldOriented, inputs.timestamp, inputs.distanceToTargets, inputs.poseAmbiguities, inputs.tagAreas)

    override fun updateInputs() {
        inputs.isConnected = camera.isConnected

        val latestResult = camera.latestResult

        if (!latestResult.hasTargets()) {
            return
        }

        val estimatedPose = estimator.update(latestResult)

        if (estimatedPose.isEmpty) {
            return
        }

        val tags = latestResult.targets

        for (tag in tags) {
            val distanceToTarget = tag.bestCameraToTarget.translation.norm
            if (distanceToTarget > VisionConstants.MAXIMUM_DISTANCE_FROM_TAG.`in`(Units.Meters)) {
                return
            }

            inputs.distanceToTargets.add(distanceToTarget)
            inputs.tagAreas.add(tag.area)
            inputs.poseAmbiguities.add(tag.poseAmbiguity)
        }

        inputs.poseFieldOriented = estimatedPose.get().estimatedPose
        inputs.timestamp = estimatedPose.get().timestampSeconds
    }
}