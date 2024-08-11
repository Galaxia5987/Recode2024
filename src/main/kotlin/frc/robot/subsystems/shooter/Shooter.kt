package frc.robot.subsystems.shooter

import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Shooter private constructor(private val io: ShooterIO) : SubsystemBase() {
    private var topVelocitySetpoint: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
    private var bottomVelocitySetpoint: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
    private val timer = Timer()
    private val subsystemName = this::class.simpleName

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

    fun setVelocity(topVelocity: Measure<Velocity<Angle>>, bottomVelocity: Measure<Velocity<Angle>>): Command {
        return run {
            topVelocitySetpoint.mut_replace(topVelocity)
            bottomVelocitySetpoint.mut_replace(bottomVelocity)
            io.setTopVelocity(topVelocity)
            io.setBottomVelocity(bottomVelocity)
        }.withName("Set Top and Bottom Velocity Command")
    }

    fun setVelocity(velocity: MutableMeasure<Velocity<Angle>>) : Command = setVelocity(velocity, velocity).withName("Set Velocity Command")

    fun stop(): Command {
        return run {
            topVelocitySetpoint.mut_replace(ShooterConstants.STOP_POWER)
            bottomVelocitySetpoint.mut_replace(ShooterConstants.STOP_POWER)
            io.stop()
        }.withName("Stop Shooter")
    }

    @AutoLogOutput
    fun atSetpoint(): Boolean =
        io.topRollerInputs.velocity.isNear(
            topVelocitySetpoint, ShooterConstants.TOP_ROLLER_TOLERANCE.`in`(Units.Percent)
        ) && io.bottomRollerInputs.velocity.isNear(
            bottomVelocitySetpoint, ShooterConstants.BOTTOM_ROLLER_TOLERANCE.`in`(
                Units.Percent
            )
        )

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("$subsystemName/TopRoller", io.topRollerInputs)
        Logger.processInputs("$subsystemName/BottomRoller", io.bottomRollerInputs)
    }
}