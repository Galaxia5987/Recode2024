package frc.robot.lib

import frc.robot.subsystems.vision.Vision

class PoseEstimation {
    private val vision: Vision = Vision.getInstance()

    fun processVisionMeasurements(multiplier: Double) {
        val results = vision.getResults()

        for (result in results) {
            val ambiguities = result.distanceToTargets.map { d -> d * d }
            val stddev = multiplier //* ambiguities // TODO: Do Harmonic Average for ambiguities list before calculating stddev

            // TODO: Update Swerve Drive Estimator with Vision Measurements
        }
    }
}