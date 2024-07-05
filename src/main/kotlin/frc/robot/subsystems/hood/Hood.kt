package frc.robot.subsystems.hood

import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Hood private constructor(private val io: HoodIO): SubsystemBase() {
    private val inputs: LoggedHoodInputs = io.inputs
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

    fun getAngle(): MutableMeasure<Angle> = inputs.internalAngle

    fun setAngle(angle: MutableMeasure<Angle>): Command {
        return run {
            inputs.angleSetpoint.mut_replace(angle)
            io.setAngle(angle)
        }.withName("Set Angle Hood")
    }

    fun setRestingAngle(): Command = setAngle(HoodConstants.RESTING_ANGLE).withName("Set Resting Angle Hood")

    override fun periodic() {
        io.updateInputs()
        if (encoderTimer.advanceIfElapsed(0.5)) io.updateInternalEncoder()
        Logger.processInputs("Hood", inputs)


    }
}