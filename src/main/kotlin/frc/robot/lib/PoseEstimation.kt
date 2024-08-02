package frc.robot.lib

import frc.robot.subsystems.vision.Vision

class PoseEstimation {
    private val vision: Vision = Vision.getInstance()

    companion object {
        @Volatile
        private var instance: PoseEstimation ?= null

        fun initialize() {
            synchronized(this) {
                if (instance == null) {
                    instance = PoseEstimation()
                }
            }
        }

        fun getInstance(): PoseEstimation {
            return instance ?: throw IllegalArgumentException(
                "PoseEst has not been initialized. Call initialize() first."
            )
        }
    }

    fun processVisionMeasurements(multiplier: Double) {
        val results = vision.getResults()

        for (result in results) {
            val ambiguities = result.distanceToTargets.map { d -> d * d }
            val stddev = multiplier //* ambiguities // TODO: Do Harmonic Average for ambiguities list before calculating stddev

            // TODO: Update Swerve Drive Estimator with Vision Measurements
        }
    }
}