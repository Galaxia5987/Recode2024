package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.units.Units
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

class PhotonVisionIOReal(private val camera: PhotonCamera, private val robotToCam: Transform3d) : VisionIO {
    private var visionResult = VisionResult()
    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(
        VisionConstants.aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        robotToCam
    )

    override val name = camera.name

    init {
        camera.pipelineIndex = 0
    }

    override fun setPipeLine(pipeLineIndex: Int) {
        camera.pipelineIndex = pipeLineIndex
    }

    override fun getLatestResult(): VisionResult = visionResult

    override fun updateResult() {
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
            visionResult.distanceToTargets.clear()
            val distanceToTarget = tag.bestCameraToTarget.translation.norm
            if (distanceToTarget > VisionConstants.MAXIMUM_DISTANCE_FROM_TAG.`in`(Units.Meters)) {
                return
            }

            visionResult.distanceToTargets.add(distanceToTarget)
        }

        visionResult.estimatedRobotPose = estimatedPose.get().estimatedPose
        visionResult.timestamp = estimatedPose.get().timestampSeconds
    }
}