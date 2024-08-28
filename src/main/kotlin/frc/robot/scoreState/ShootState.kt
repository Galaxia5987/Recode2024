package frc.robot.scoreState

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.commandGroups.WarmupCommands
import frc.robot.lib.PoseEstimation
import frc.robot.lib.getRotationToTranslation
import frc.robot.lib.handleInterrupt
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveDrive

class ShootState : ScoreState {
    private val swerveDrive = SwerveDrive.getInstance()
    private val shooter = Shooter.getInstance()
    private val hood = Hood.getInstance()
    private val gripper = Gripper.getInstance()


    private fun getRotationToSpeaker(): Rotation2d {
        return swerveDrive.estimator.estimatedPosition.translation.getRotationToTranslation(
            Constants.SPEAKER_POSE
        )
    }

    private fun warmup(distanceToSpeaker: Measure<Distance>): Command {
        return WarmupCommands.warmup(
            Units.Degrees.of(
                ScoreConstants.HOOD_ANGLE_BY_DISTANCE.getInterpolated(
                    InterpolatingDouble(distanceToSpeaker.`in`(Units.Meters))
                ).value
            ), Units.RotationsPerSecond.of(
                ScoreConstants.SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(
                    InterpolatingDouble(distanceToSpeaker.`in`(Units.Meters))
                ).value
            ), Units.RotationsPerSecond.of(
                ScoreConstants.CONVEYOR_VELOCITY_BY_DISTANCE.getInterpolated(
                    InterpolatingDouble(distanceToSpeaker.`in`(Units.Meters))
                ).value
            )
        )
    }

    private fun turnToSpeaker(): Command {
        return swerveDrive.turnCommand(
            Units.Rotations.of(
                getRotationToSpeaker().rotations
            ), 3.0 / 360.0
        )
    }

    private fun readyToShoot(): Boolean {
        return shooter.atSetpoint() && hood.atSetpoint() && swerveDrive.atTurnSetpoint
    }

    private fun init(): Command {
        return Commands.parallel(
            warmup(Units.Meters.of(PoseEstimation.getInstance().getDistanceToSpeaker())), turnToSpeaker()
        ).until(::readyToShoot).andThen(Commands.none()) //TODO: Replace with LEDs command
    }

    private fun end(): Command {
        return gripper.feed().andThen(
            WarmupCommands.stopWarmup(), Commands.none() //TODO: Replace with LEDs command
        )
    }

    override fun execute(): Command {
        return init().handleInterrupt(end())
    }
}