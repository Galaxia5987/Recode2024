package frc.robot.subsystems.shooter

import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Shooter private constructor(private val io: ShooterIO) : SubsystemBase() {
    private val topRollerInputs = io.topRollerInputs
    private val bottomRollerInputs = io.bottomRollerInputs
    private val timer = Timer()
    private val SUBSYSTEM_NAME = this::class.simpleName

    companion object {
        @Volatile
        private var instance: Shooter? = null

        fun initialize(io: ShooterIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Shooter(io)
                }
            }
        }

        fun getInstance() : Shooter {
            return instance ?: throw IllegalArgumentException(
                "Shooter has not been initialized. Call initialize(io: ShooterIO) first."
            )
        }
    }

    init {
        timer.start()
        timer.reset()
    }

    fun setVelocity(topVelocity: MutableMeasure<Velocity<Angle>>, bottomVelocity: MutableMeasure<Velocity<Angle>>): Command {
        return run {
            topRollerInputs.velocity.mut_replace(topVelocity)
            bottomRollerInputs.velocity.mut_replace(bottomVelocity)
            io.setTopVelocity(topVelocity)
            io.setBottomVelocity(bottomVelocity)
        }.withName("") // TODO:
    }

    fun setVelocity(velocity: MutableMeasure<Velocity<Angle>>) : Command = setVelocity(velocity, velocity)

    fun stop(): Command {
        return run {
            topRollerInputs.velocitySetpoint.mut_replace(ShooterConstants.STOP_POWER)
            bottomRollerInputs.velocitySetpoint.mut_replace(ShooterConstants.STOP_POWER)
            io.stop()
        }.withName("Stop Shooter")
    }

    @AutoLogOutput
    fun atSetpoint(): Boolean = topRollerInputs.velocity.isNear(topRollerInputs.velocitySetpoint, ShooterConstants.TOP_ROLLER_TOLERANCE.`in`(
        Units.RotationsPerSecond)) && bottomRollerInputs.velocity.isNear(bottomRollerInputs.velocitySetpoint, ShooterConstants.BOTTOM_ROLLER_TOLERANCE.`in`(
        Units.RotationsPerSecond))

    override fun periodic() {
        io.updateInputs()
        if (timer.advanceIfElapsed(0.1)) {
            Logger.processInputs("$SUBSYSTEM_NAME/TopRoller", topRollerInputs)
            Logger.processInputs("$SUBSYSTEM_NAME/BottomRoller", bottomRollerInputs)
        }
    }
}