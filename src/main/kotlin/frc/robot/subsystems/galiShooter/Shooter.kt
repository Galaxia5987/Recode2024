package frc.robot.subsystems.galiShooter

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Shooter(private val io: ShooterIO) : SubsystemBase() {
    private val topInputs = io.topInputs
    private val bottomInputs = io.bottomInputs

    fun setTopVelocity(velocity: Measure<Velocity<Angle>>): Command {
        return Commands.run({
            topInputs.velocitySetpoint = velocity
            io.setTopVelocity(velocity)
            }
        )
    }

    fun setBottomVelocity(velocity: Measure<Velocity<Angle>>): Command {
        return Commands.run({
            bottomInputs.velocitySetpoint = velocity
            io.setBottomVelocity(velocity)
            }
        )
    }

    fun stop(velocity: Measure<Velocity<Angle>>): Command {
        return Commands.run({
            io.stop()
            }
        )
    }

    fun setVelocity(topVelocity: Measure<Velocity<Angle>>, bottomVelocity: Measure<Velocity<Angle>>): Command {
        return Commands.parallel(
            setTopVelocity(topVelocity),
            setBottomVelocity(bottomVelocity)
        )
    }

    fun setVelocity(velocity: Measure<Velocity<Angle>>): Command {
        return setVelocity(topVelocity = velocity, bottomVelocity = velocity)
    }

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName + "/topInputs", topInputs)
        Logger.processInputs(this::class.simpleName + "/bottomInputs", bottomInputs)
    }
}