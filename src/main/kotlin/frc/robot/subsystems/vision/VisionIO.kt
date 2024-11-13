package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import org.team9432.annotation.Logged

interface VisionIO {
    val inputs: LoggedVisionInputs

    val name: String

    fun setPipeLine(pipeLineIndex: Int) {
    }

    fun updateInputs() {}

    @Logged
    open class VisionInputs {
        var poseFieldOriented = Pose3d()
        var timestamp: Double = 0.0 // Seconds
        var bestCameraToTargets: MutableList<Transform3d> = ArrayList()
    }
}
