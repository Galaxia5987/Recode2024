package frc.robot.lib

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.Constants
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive
import frc.robot.subsystems.vision.Vision
import org.littletonrobotics.junction.AutoLogOutput
import kotlin.math.sqrt

class PoseEstimation {
    private val vision: Vision = Vision.getInstance()
    private val swerveDrive: SwerveDrive = SwerveDrive.getInstance()

    companion object {
        @Volatile
        private var instance: PoseEstimation? = null

        fun initialize() {
            synchronized(this) {
                if (instance == null) {
                    instance = PoseEstimation()
                }
            }
        }

        fun getInstance(): PoseEstimation {
            return instance ?: throw IllegalArgumentException(
                "PoseEstimation has not been initialized. Call initialize() first."
            )
        }
    }

    private fun isOutOfBounds(estimatedPose: Pose3d): Boolean {
        val x = estimatedPose.x
        val y = estimatedPose.y
        return !(0 < x && x < Constants.FIELD_LENGTH) || !(0 < y && y < Constants.FIELD_WIDTH)
    }

    fun processVisionMeasurements(multiplier: Double) {
        val results = vision.results

        for (result in results) {
            val estimatedPose = result.estimatedRobotPose

            val isFloating = estimatedPose.z > 0.1
            val isOutOfBounds = isOutOfBounds(estimatedPose)

            if (isFloating || isOutOfBounds) {
                continue
            }

            val distances = result.distanceToTargets
            val mean = distances.average()

            val stddev = if (distances.size == 1) {
                distances.first() * multiplier
            } else {
                val squaredDistances = distances.map { d -> (d - mean) * (d - mean) }
                val variance = squaredDistances.average()
                sqrt(variance)
            }

            swerveDrive.estimator.addVisionMeasurement(
                estimatedPose.toPose2d(),
                result.timestamp,
                VecBuilder.fill(
                    stddev,
                    stddev,
                    stddev * SwerveConstants.MAX_OMEGA_VELOCITY / SwerveConstants.MAX_X_Y_VELOCITY
                )
            )
        }
    }
}