package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import org.team9432.annotation.Logged

interface VisionIO {
    val name: String

    fun setPipeLine(pipeLineIndex: Int) {

    }

    fun getLatestResult(): VisionResult = VisionResult()

    fun updateResult() {}
}