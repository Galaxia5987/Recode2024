package frc.robot.commandGroups

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.StartEndCommand
import frc.robot.subsystems.climb.Climb

object CommandGroups {
    private val climb = Climb.getInstance()

    fun openClimb(): Command{
        return Commands.sequence(
            climb.setPower {-0.3}.withTimeout(1.25),
            climb.open(),
            climb.setPower {-0.5}.withTimeout(2.5),
        )
    }

    fun closeClimb(): Command{
        return StartEndCommand(
            {climb.setPower {0.5}},
            climb::lock,
            climb
        )
    }
}