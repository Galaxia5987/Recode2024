package frc.robot.scoreState

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.commandGroups.WarmupCommands
import frc.robot.lib.*
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveDrive
import org.littletonrobotics.junction.AutoLogOutput

class ShootState : ScoreState {
    private val swerveDrive = SwerveDrive.getInstance()
    private val shooter = Shooter.getInstance()
    private val hood = Hood.getInstance()
    private val gripper = Gripper.getInstance()
    private val hoodAngle: LoggedTunableNumber = LoggedTunableNumber("HoodAngle", 60.0)
    private val shooterVelocity: LoggedTunableNumber = LoggedTunableNumber("ShooterVelocity", 60.0)
    private val conveyorVelocity: LoggedTunableNumber = LoggedTunableNumber("ConveyorVelocity", 60.0)


    private fun getRotationToSpeaker(): Rotation2d {
        return swerveDrive.estimator.estimatedPosition.translation.getRotationToTranslation(
            Constants.SPEAKER_POSE
        )
    }

    private fun warmup(distanceToSpeaker: Measure<Distance>): Command {
        return Commands.defer({WarmupCommands.warmup(
            Units.Degrees.of(
                hoodAngle.get()
//                ScoreConstants.HOOD_ANGLE_BY_DISTANCE.getInterpolated(
//                    InterpolatingDouble(distanceToSpeaker.`in`(Units.Meters))
//                ).value
            ), Units.RotationsPerSecond.of(
                shooterVelocity.get()
//                ScoreConstants.SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(
//                    InterpolatingDouble(distanceToSpeaker.`in`(Units.Meters))
//                ).value
            ), Units.RotationsPerSecond.of(
                conveyorVelocity.get()
//                ScoreConstants.CONVEYOR_VELOCITY_BY_DISTANCE.getInterpolated(
//                    InterpolatingDouble(distanceToSpeaker.`in`(Units.Meters))
//                ).value
            )
        )}, setOf(hood, shooter, Conveyor.getInstance()))
    }

    private fun turnToSpeaker(): Command {
        return swerveDrive.turnCommand(
            Units.Rotations.of(
                getRotationToSpeaker().rotations
            ), 5.0 / 360.0
        )
    }

    @AutoLogOutput
    private fun readyToShoot(): Boolean {
        return shooter.atSetpoint() && hood.atSetpoint() && swerveDrive.atTurnSetpoint
    }

    private fun init(): Command {
        return Commands.parallel(
            warmup(Units.Meters.of(Robot.getDistanceToSpeaker())),
//            turnToSpeaker()
        ).until(::readyToShoot).andThen(Commands.none()) //TODO: Replace with LEDs command
    }

    private fun end(): Command {
        return gripper.feed().andThen(
            Commands.parallel(
                shooter.stop(), Conveyor.getInstance().stop(), hood.setRestingAngle() //TODO: Replace with LEDs command
            )
        )
    }

    override fun execute(): Command {
        return init().finallyDo(end())
    }
}