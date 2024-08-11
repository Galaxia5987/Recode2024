package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import kotlin.math.absoluteValue

class Climb private constructor(private val io: ClimbIO) : SubsystemBase() {
    private val inputs = io.inputs
    private val timer = Timer()

    @AutoLogOutput
    private var isStopperStuck = false

    companion object {
        @Volatile
        private var instance: Climb? = null

        fun initialize(io: ClimbIO) {
            synchronized(this) {
                if (instance == null)
                    instance = Climb(io)
            }
        }

        fun getInstance(): Climb {
            return instance ?: throw IllegalStateException("climb not initialized, call initialize(io)")
        }
    }

    init {
        timer.start()
        timer.reset()
    }

    fun open(): Command {
        return Commands.run(io::openStopper)
            .until { isStopperStuck }
            .andThen(io::disableStopper)
    }

    fun lock(): Command {
        return Commands.run(io::closeStopper)
            .until { isStopperStuck }
            .andThen(io::disableStopper)
    }

    fun setPower(power: DoubleSupplier): Command {
        return run { io.setPower(power.asDouble) }
    }

    override fun periodic() {
        isStopperStuck =
            inputs.stopperCurrent.absoluteValue >=
                    ClimbConstants.STOPPER_MOTOR_CURRENT_THRESHOLD.absoluteValue
        io.updateInputs()
        if (timer.advanceIfElapsed(0.1)) {
            Logger.processInputs(this::class.simpleName, inputs)
        }
    }
}