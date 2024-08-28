package frc.robot.scoreState

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.StartEndCommand
import frc.robot.Constants
import frc.robot.lib.handleInterrupt
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.swerve.SwerveDrive

class ClimbState : ScoreState {

    private val swerveDrive = SwerveDrive.getInstance()
    private val climb = Climb.getInstance()

    private fun nearestChain(): Pose2d {
        return swerveDrive.estimator.estimatedPosition.nearest(Constants.CHAIN_LOCATIONS)
    }

    private fun pathFindToChain(): Command {
        return Commands.parallel(
            AutoBuilder.pathfindToPose(nearestChain(), Constants.PATH_CONSTRAINTS), swerveDrive.turnCommand(
                Units.Rotations.of(
                    nearestChain().rotation.rotations
                ), 2.0 / 360
            )
        )
    }

    override fun execute(): Command {
        return Commands.sequence(
            climb.openClimb(), pathFindToChain(), climb.closeClimb()
        ).handleInterrupt(Commands.none()) // TODO: Replace with LEDs command
    }
}