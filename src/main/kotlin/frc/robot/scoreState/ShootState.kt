package frc.robot.scoreState

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.ControllerInputs
import frc.robot.Robot
import frc.robot.commandGroups.WarmupCommands
import frc.robot.lib.finallyDo
import frc.robot.lib.getRotationToTranslation
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive
import org.littletonrobotics.junction.AutoLogOutput

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

    private fun warmup(): Command {
        return WarmupCommands.warmup(
            {
                Units.Degrees.of(
                    ScoreConstants.HOOD_ANGLE_BY_DISTANCE.getInterpolated(
                        InterpolatingDouble(Robot.getDistanceToSpeaker())
                    ).value
                )
            },
            {
                Units.RotationsPerSecond.of(
                    ScoreConstants.SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(
                        InterpolatingDouble(Robot.getDistanceToSpeaker())
                    ).value
                )
            },
            {
                Units.RotationsPerSecond.of(
                    ScoreConstants.CONVEYOR_VELOCITY_BY_DISTANCE.getInterpolated(
                        InterpolatingDouble(Robot.getDistanceToSpeaker())
                    ).value
                )
            }
        )
    }

    private fun turnToSpeaker(): Command {
        return swerveDrive.driveAndAdjust(
            {
                Units.Rotations.of(
                    getRotationToSpeaker().rotations
                )
            },
            { -ControllerInputs.driverController().leftY },
            { -ControllerInputs.driverController().leftX },
            SwerveConstants.SHOOT_TURN_TOLERANCE,
            0.1,
            true
        )
    }

    @AutoLogOutput
    private fun readyToShoot(): Boolean {
        return shooter.atSetpoint() && hood.atSetpoint() && swerveDrive.atTurnSetpoint
    }

    fun init(): Command {
        return Commands.parallel(
            warmup(),
            turnToSpeaker(),
            Commands.none().onlyIf(::readyToShoot) //TODO: Replace with LEDs command
        )
    }

    fun end(): Command {
        return gripper.feed().andThen(
            Commands.parallel(
                shooter.stop(), Conveyor.getInstance().stop(), hood.setRestingAngle() //TODO: Replace with LEDs command
            )
        )
    }

    override fun execute(): Command {
        return init().finallyDo(end()).withName("Speaker")
    }
}