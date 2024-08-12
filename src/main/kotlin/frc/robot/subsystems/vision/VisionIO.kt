package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import org.team9432.annotation.Logged

interface VisionIO {
    val inputs: LoggedVisionInputs

    fun setPipeLine(pipeLineIndex: Int) {

    }

    fun getLatestResult(): VisionResult = VisionResult()

    fun updateInputs() {}

    @Logged
    open class VisionInputs {
        var poseFieldOriented = Pose3d()
        var isConnected = false
        var timestamp: Double = 0.0 // Seconds
        var distanceToTargets: MutableList<Double> = ArrayList()
        var poseAmbiguities: MutableList<Double> = ArrayList()
        var tagAreas: MutableList<Double> = ArrayList() // Area is in percentages
    }
}