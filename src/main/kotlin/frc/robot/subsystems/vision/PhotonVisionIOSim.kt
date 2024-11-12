package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Transform3d
import frc.robot.lib.toPose3d
import frc.robot.subsystems.swerve.SwerveDrive
import org.photonvision.PhotonPoseEstimator
import org.photonvision.estimation.TargetModel
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.VisionTargetSim

class PhotonVisionIOSim(private val simCamera: PhotonCameraSim, private val robotToCam: Transform3d) : VisionIO {
    override val inputs = LoggedVisionInputs()
    private val estimator: PhotonPoseEstimator = PhotonPoseEstimator(
        aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        simCamera.camera,
        robotToCam
    )

    override val name = simCamera.camera.name


    init {
        VisionSim.system.addAprilTags(aprilTagFieldLayout)
        VisionSim.system.addCamera(simCamera, robotToCam)
    }


    override fun updateInputs() {
        val botPose = SwerveDrive.getInstance().estimator.estimatedPosition
        val botPose3d = botPose.toPose3d()
        val aprilTags = aprilTagFieldLayout.tags.map {
            VisionTargetSim(
                it.pose,
                TargetModel.kAprilTag36h11,
                it.ID
            )
        }
        val latestResult =
            simCamera.process(
                0.0,
                botPose3d + robotToCam.inverse(),
                aprilTags
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
        }
    }
}