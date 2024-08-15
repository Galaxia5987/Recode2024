package frc.robot.scoreState

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.StartEndCommand
import frc.robot.Constants
import frc.robot.commandGroups.CommandGroups
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.swerve.SwerveDrive

class ClimbState: ScoreState {

    private val swerveDrive = SwerveDrive.getInstance()
    private val botPose = Pose2d() //Todo: replace with estimator

    private fun getNearestChain(): Pose2d {
        return botPose.nearest(Constants.chainLocations.asList())
    }

    private fun pathFindToPose(pose: Pose2d): Command{
        return Commands.parallel(
            AutoBuilder.pathfindToPose(pose, Constants.PATH_CONSTRAINTS),
            swerveDrive.turnCommand(
                Units.Rotations.of(
                getNearestChain().rotation.rotations).mutableCopy(),
                2.0/360
            )
        )
    }

    override fun execute(): Command {
        val optimalPose: Pose2d = getNearestChain()
        return StartEndCommand(
            {Commands.sequence(
                CommandGroups.openClimb(),
                pathFindToPose(optimalPose),
                CommandGroups.closeClimb()
            )},
            Commands::none, //TODO: Replace with LEDs command
            swerveDrive, Climb.getInstance())
    }
}