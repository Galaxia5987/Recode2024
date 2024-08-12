package frc.robot.scoreState

import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.StartEndCommand
import frc.robot.Constants
import frc.robot.commandGroups.CommandGroups
import frc.robot.lib.extensions.TranslationExtensions.getRotationToTranslation
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveDrive

class ShootState : ScoreState {
    private val swerveDrive = SwerveDrive.getInstance()
    private val shooter = Shooter.getInstance()
    private val hood = Hood.getInstance()
    private val conveyor = Conveyor.getInstance()
    private val gripper = Gripper.getInstance()

    val botPose = Pose2d()

    fun getDistanceToSpeaker(): Measure<Distance> {
        return when (Constants.alliance) {
            Constants.Alliance.RED -> Units.Meters.of(
                botPose.translation.getDistance(
                    GeometryUtil.flipFieldPosition(
                        ScoreConstants.SPEAKER_POSE_BLUE
                    )
                )
            )

            Constants.Alliance.BLUE -> Units.Meters.of(
                botPose.translation.getDistance(
                    ScoreConstants.SPEAKER_POSE_BLUE
                )
            )
        }
    }

    fun getRotationToSpeaker(): Rotation2d {
        return when (Constants.alliance) {
            Constants.Alliance.RED ->
                botPose.translation.getRotationToTranslation(
                    GeometryUtil.flipFieldPosition(
                        ScoreConstants.SPEAKER_POSE_BLUE
                    )
                )

            Constants.Alliance.BLUE ->
                botPose.translation.getRotationToTranslation(
                    ScoreConstants.SPEAKER_POSE_BLUE
                )
        }
    }

    private fun warmup(distanceToSpeaker: Measure<Distance>): Command {
        return CommandGroups.warmup(
            Units.Degrees.of(
                ScoreConstants.HOOD_ANGLE_BY_DISTANCE.getInterpolated(
                    InterpolatingDouble(distanceToSpeaker.`in`(Units.Meters))
                ).value
            ),
            Units.RotationsPerSecond.of(
                ScoreConstants.SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(
                    InterpolatingDouble(distanceToSpeaker.`in`(Units.Meters))
                ).value
            ),
            Units.RotationsPerSecond.of(
                ScoreConstants.CONVEYOR_VELOCITY_BY_DISTANCE.getInterpolated(
                    InterpolatingDouble(distanceToSpeaker.`in`(Units.Meters))
                ).value
            )
        )
    }

    fun turnToSpeaker(): Command {
        return swerveDrive.turnCommand(
            Units.Rotations.of(
                getRotationToSpeaker().rotations
            ).mutableCopy(),
            3.0 / 360.0
        )
    }

    fun readyToShoot(): Boolean {
        return shooter.atSetpoint() && hood.atSetpoint() && swerveDrive.atTurnSetpoint
    }

    fun init(): Runnable {
        return Runnable {
            Commands.parallel(
                warmup(getDistanceToSpeaker()),
                turnToSpeaker()
            ).until(::readyToShoot).andThen(Commands.none()) //TODO: Replace with LEDs command
        }
    }

    fun end(): Runnable {
        return Runnable {
            gripper.feed()
                .andThen(
                    CommandGroups.stopWarmup(),
                    Commands.none() //TODO: Replace with LEDs command
                )
        }
    }

    override fun execute(): Command {
        return StartEndCommand(
            init(),
            end(),
            swerveDrive, shooter, hood, conveyor, gripper
        )
    }
}