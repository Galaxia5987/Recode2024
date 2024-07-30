package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import org.team9432.annotation.Logged

interface VisionIO {
    val inputs: LoggedVisionInputs

    fun setPipeLine(pipeLineIndex: Int) {

    }

    fun getLatestResult(): VisionResult? = null

    fun updateInputs() {}

    @Logged
    open class VisionInputs {
        var poseFieldOriented = Pose3d()
        var isConnected = false
        var timestamp: Double = 0.0 // Seconds
        var ambiguity: Double = 0.0
    }
}