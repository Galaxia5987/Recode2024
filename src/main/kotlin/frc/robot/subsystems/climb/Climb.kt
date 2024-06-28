package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Climb(private val io: ClimbIO): SubsystemBase() {
    private val inputs = LoggedClimbInputs()

    fun open():Command{
        return Commands.run({io.closeStopper()}).until{io.isStopperStuck}
    }

    fun lock():Command{
        return Commands.run({io.closeStopper()}).until{io.isStopperStuck}
    }
}