package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import org.team9432.annotation.Logged

interface VisionIO {

    fun setPipeLine(pipeLineIndex: Int) {}

    fun getLastestResult() : VisionResult {
        throw NotImplementedError("Method getLatestResult Not Implemented in current VisionIO")
    }

    fun getName(): String = "Camera"

    fun updateInputs() {}


    @Logged
    open class VisionInputs {
        var poseFieldOriented = Pose3d()
        var isConnected = false
    }
}