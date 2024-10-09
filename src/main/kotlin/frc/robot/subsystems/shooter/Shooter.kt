package frc.robot.subsystems.shooter

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class Shooter private constructor(private var io: ShooterIO) {
    private var input = io.inputs

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

    fun setTopVel(vel: Measure<Velocity<Angle>>): Command = Commands.runOnce({ io.setTopVel(vel) })
    fun setBottomVel(vel: Measure<Velocity<Angle>>): Command = Commands.runOnce({ io.setBottomVel(vel) })

    fun stop(): Command = Commands.runOnce({
        io.setBottomVel(MutableMeasure.zero(Units.RotationsPerSecond))
        io.setTopVel(MutableMeasure.zero(Units.RotationsPerSecond))
    })

    fun setShooterVel(vel: Measure<Velocity<Angle>>): Command =
        Commands.runOnce({
            setTopVel(vel)
            setBottomVel(vel)
        })
}