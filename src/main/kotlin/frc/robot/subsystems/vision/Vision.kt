package frc.robot.subsystems.vision

import edu.wpi.first.wpilibj2.command.SubsystemBase

class Vision private constructor(private val ios: List<VisionIO>) : SubsystemBase() {
    private var results: MutableList<VisionResult> = ArrayList()

    companion object {
        @Volatile
        private var instance: Vision? = null

        fun initialize(ios: List<VisionIO>) {
            synchronized(this) {
                if (instance == null) {
                    instance = Vision(ios)
                }
            }
        }

        fun getInstance(): Vision {
            return instance ?: throw IllegalArgumentException(
                "Vision has not been initialized. Call initialize(ios: List<VisionIO>) first."
            )
        }
    }

    fun getResults(): List<VisionResult> = results

    override fun periodic() {
        results.clear()
        for (io: VisionIO in ios) {
            val latestResult = io.getLatestResult()
            if (latestResult != null) {
                results.add(latestResult)
            }
        }
    }
}