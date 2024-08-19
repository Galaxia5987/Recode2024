package frc.robot.scoreState

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.StartEndCommand
import frc.robot.ControllerInputs
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveDrive

class AmpState : ScoreState {
    private val swerveDrive: SwerveDrive = SwerveDrive.getInstance()
    private val shooter: Shooter = Shooter.getInstance()
    private val conveyor: Conveyor = Conveyor.getInstance()
    private val hood: Hood = Hood.getInstance()
    private val gripper: Gripper = Gripper.getInstance()

    private fun init(): Runnable {
        val driveAndAdjust = swerveDrive.driveAndAdjust(
            ScoreConstants.AMP_ROTATION,
            { -ControllerInputs.getDriverController().leftX },
            { -ControllerInputs.getDriverController().leftY },
            0.1,
            false
        )
        val setShooterVelocity = shooter.setVelocity(
            ScoreConstants.SHOOTER_TOP_AMP_VELOCITY,
            ScoreConstants.SHOOTER_BOTTOM_AMP_VELOCITY
        )
        return Runnable {
            Commands.parallel(
                driveAndAdjust, setShooterVelocity, conveyor.setVelocity(ScoreConstants.CONVEYOR_AMP_VELOCITY)
                // TODO: Add LEDS
            )
        }
    }

    private fun end(): Runnable {
        return Runnable {
            gripper.feed()
                .andThen(shooter.stop())
                .alongWith(conveyor.stop())
        }
    }

    override fun execute(): Command {
        return StartEndCommand(
            { init() },
            { end() },
            swerveDrive, shooter, conveyor, hood, gripper
        )
    }
}