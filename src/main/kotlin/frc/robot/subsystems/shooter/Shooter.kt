package frc.robot.subsystems.shooter

import edu.wpi.first.units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
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

    fun setTopVelocity(velocity: AngularVelocity): Command = Commands.runOnce({ io.setTopVelocity(velocity) })
    fun setBottomVelocity(velocity: AngularVelocity): Command =
        Commands.runOnce({ io.setBottomVelocity(velocity) })

    fun stop(): Command = Commands.runOnce({
        io.setBottomVelocity(Units.RotationsPerSecond.zero())
        io.setTopVelocity(Units.RotationsPerSecond.zero())
    })

    fun setShooterVel(vel: AngularVelocity): Command =
        Commands.runOnce({
            setTopVelocity(vel)
            setBottomVelocity(vel)
        })

    val routine = SysIdRoutine(
        SysIdRoutine.Config(),
        Mechanism(
            { inputs.topVoltage },
            { SysIdRoutineLog(":)") },
            this
        )
    )

    fun sysIdQuasistatic(direction: SysIdRoutine.Direction): Command {
        return routine.quasistatic(direction)
    }

    fun sysIdDynamic(direction: SysIdRoutine.Direction): Command {
        return routine.dynamic(direction)
    }


    override fun periodic() {
        io.updateInput()
        Logger.processInputs(this::class.simpleName, inputs)
    }
}