package frc.robot.subsystems.conveyor

import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Conveyor private constructor(private val io: ConveyorIO) : SubsystemBase() {
    @AutoLogOutput
    private var velocitySetpoint: Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()
    private val inputs = io.inputs
    private val timer = Timer()

    companion object {
        private var instance: Conveyor? = null

        fun initialize(io: ConveyorIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Conveyor(io)
                }
            }
        }

        fun getInstance(): Conveyor {
            return instance ?: throw IllegalArgumentException(
                "Conveyor has not been initialized. Call initialize(io: HoodIO) first."
            )
        }
    }

    init {
        timer.start()
        timer.reset()
    }

    fun setVelocity(velocity: Measure<Velocity<Angle>>): Command = run {
        velocitySetpoint = velocity
        io.setVelocity(velocity)
    }

    fun feed() = setVelocity(ConveyorConstants.FEED_VELOCITY)

    @AutoLogOutput
    fun atSetPoint(): Boolean {
        return inputs.velocity.isNear(velocitySetpoint, ConveyorConstants.AT_SETPOINT_TOLERANCE.`in`(Units.Percent))
    }

    fun stop(): Command {
        return runOnce {
            velocitySetpoint = Units.RotationsPerSecond.zero()
            io.stop()
        }
    }

    override fun periodic() {
        io.updateInputs()
        if (timer.advanceIfElapsed(0.1)) Logger.processInputs(this::class.simpleName, inputs)
    }
}