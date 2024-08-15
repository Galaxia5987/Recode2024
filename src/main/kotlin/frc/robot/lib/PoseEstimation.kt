package frc.robot.lib

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.Constants
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive
import frc.robot.subsystems.vision.Vision
import org.littletonrobotics.junction.AutoLogOutput

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

        /**
         * Averages ambiguity of estimated poses using a harmonic average. Can be from different targets
         * in vision module, or between module.
         *
         * Note: The numerator is 1 instead of the usual 'n'.
         * This is so when we have multiple lower values the ambiguity will be lower.
         *
         * @param ambiguities the ambiguities to average.
         * @return the average of the ambiguities.
         */
        fun averageAmbiguity(ambiguities: List<Double>): Double {
            return 1.0 / ambiguities.sumOf { 1.0 / it }
        }
    }

    fun processVisionMeasurements(multiplier: Double) {
        val results = vision.results

        for (result in results) {
            val ambiguities = result.distanceToTargets.map { d -> d * d }
            val stddev = multiplier * averageAmbiguity(ambiguities)

            swerveDrive.estimator.addVisionMeasurement(
                result.estimatedRobotPose.toPose2d(),
                result.timestamp,
                VecBuilder.fill(
                    stddev,
                    stddev,
                    stddev * SwerveConstants.MAX_OMEGA_VELOCITY / SwerveConstants.MAX_X_Y_VELOCITY
                )
            )
        }
    }

    @AutoLogOutput(key = "Robot/DistanceToSpeakerk")
    fun getDistanceToSpeaker(): Double = getPoseRelativeToSpeaker().norm

    @AutoLogOutput(key = "Robot/ToSpeaker")
    fun getPoseRelativeToSpeaker(): Translation2d =
        Constants.SPEAKER_POSE.minus(
            SwerveDrive.getInstance().estimator.estimatedPosition.translation
        )
}