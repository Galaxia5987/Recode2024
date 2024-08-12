package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

class PhotonVisionIOReal(private val camera: PhotonCamera, private val robotToCam: Transform3d) : VisionIO {
    override val inputs = LoggedVisionInputs()
    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam)

    init {
        camera.pipelineIndex = 0
    }

    override fun setPipeLine(pipeLineIndex: Int) {
        camera.pipelineIndex = pipeLineIndex
    }

    override fun getLatestResult(): VisionResult = VisionResult(inputs.poseFieldOriented, inputs.timestamp, inputs.distanceToTargets, inputs.poseAmbiguities)

    override fun updateInputs() {
        inputs.isConnected = camera.isConnected

        val latestResult = camera.latestResult

        if (!latestResult.hasTargets()) {
            return
        }

        val estimatedPose = estimator.update(latestResult)

        inputs.poseFieldOriented = estimatedPose.get().estimatedPose
        inputs.timestamp = estimatedPose.get().timestampSeconds

        val tags = latestResult.targets

        for (tag in tags) {
            inputs.distanceToTargets.add(tag.bestCameraToTarget.translation.norm)
            inputs.poseAmbiguities.add(tag.poseAmbiguity)
        }

    }
}