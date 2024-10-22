package frc.robot.scoreState

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.Robot
import frc.robot.commandGroups.ShootingCommands
import frc.robot.commandGroups.WarmupCommands
import frc.robot.lib.finallyDo
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.leds.LEDConstants
import frc.robot.subsystems.leds.LEDs
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveDrive
import org.littletonrobotics.junction.AutoLogOutput

class ShootState : ScoreState {
    private val swerveDrive = SwerveDrive.getInstance()
    private val shooter = Shooter.getInstance()
    private val hood = Hood.getInstance()
    private val gripper = Gripper.getInstance()
    private val leds = LEDs.getInstance()

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

    @AutoLogOutput
    private fun readyToShoot(): Boolean {
        return shooter.atSetpoint() && hood.atSetpoint() && swerveDrive.atTurnSetpoint
    }

    fun init(): Command {
        return Commands.parallel(
            warmup(),
            ShootingCommands.turnToSpeaker(),
            leds.setSolidMode(LEDConstants.READY_TO_SHOOT_COLOR).onlyIf(::readyToShoot)
        )
    }

    fun end(): Command {
        return gripper.feed().andThen(
            Commands.parallel(
                shooter.stop(), Conveyor.getInstance().stop(),
                hood.setRestingAngle(), leds.setSolidMode(LEDConstants.SHOOT_STATE_COLOR)
            )
        )
    }

    override fun execute(): Command {
        return init().finallyDo(end()).withName("Speaker")
    }
}