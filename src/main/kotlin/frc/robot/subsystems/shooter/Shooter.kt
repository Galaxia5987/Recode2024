package frc.robot.subsystems.shooter

import edu.wpi.first.units.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Shooter private constructor(private var io: ShooterIO) : SubsystemBase() {
    private val inputs = io.inputs

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

        fun getInstance(): Shooter {
            return instance ?: throw IllegalStateException(
                "Shooter has not been initialized. Call initialize(io: shoter) first."
            )
        }
    }

    fun setTopVelocity(velocity: Measure<Velocity<Angle>>): Command = Commands.runOnce({ io.setTopVelocity(velocity) })
    fun setBottomVelocity(velocity: Measure<Velocity<Angle>>): Command = Commands.runOnce({ io.setBottomVelocity(velocity) })

    fun stop(): Command = Commands.runOnce({
        io.setBottomVelocity(MutableMeasure.zero(Units.RotationsPerSecond))
        io.setTopVelocity(MutableMeasure.zero(Units.RotationsPerSecond))
    })

    fun setShooterVel(vel: Measure<Velocity<Angle>>): Command =
        Commands.runOnce({
            setTopVelocity(vel)
            setBottomVelocity(vel)
        })

    override fun periodic() {
        io.updateInput()
        Logger.processInputs(this::class.simpleName, inputs)
    }
}