package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.DoubleSupplier

class Climb private constructor(private val io: ClimbIO) : SubsystemBase() {
    private val inputs = LoggedClimbInputs()

    companion object {
        @Volatile
        private var instance: Climb? = null

        fun initialize(io: ClimbIO){
            synchronized(this){
                if (instance==null)
                    instance = Climb(io)
            }
        }
    }

    fun open(): Command {
        return Commands.run({ io.closeStopper() })
            .until { inputs.isStopperStuck }
            .andThen({ io.disableStopper() })
    }

    fun lock(): Command {
        return Commands.run({ io.closeStopper() })
            .until { inputs.isStopperStuck }
            .andThen({ io.disableStopper() })
    }

    fun setPower(power: DoubleSupplier): Command {
        return Commands.run({ io.setPower(power.asDouble) })
    }
}