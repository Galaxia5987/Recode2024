package frc.robot.subsystems.hood

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Hood private constructor(io: HoodIO): SubsystemBase() {
    private val inputs: LoggedHoodInputs = HoodIO.inputs
    private val timer = Timer()
    private val encoderTimer = Timer()

    companion object { // Custom Singleton Implementation
        @Volatile
        private var instance: Hood? = null

        fun initialize(io: HoodIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Hood(io)
                }
            }
        }

        fun getInstance(): Hood {
            return instance ?: throw IllegalStateException(
                "Hood has not been initialized. Call initialize(io: HoodIO) first."
            )
        }
    }

    init {
        timer.start()
        timer.reset()

        encoderTimer.start()
        encoderTimer.reset()
    }

    fun atSetpoint() : Boolean = inputs.absoluteEncoderAngle.isNear(inputs.angleSetpoint, HoodConstants.MAX_TOLERANCE_DEG)
}