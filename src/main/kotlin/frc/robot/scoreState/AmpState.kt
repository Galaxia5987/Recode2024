package frc.robot.scoreState

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.ControllerInputs
import frc.robot.lib.handleInterrupt
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveConstants
import frc.robot.subsystems.swerve.SwerveDrive

class AmpState : ScoreState {
    private val swerveDrive: SwerveDrive = SwerveDrive.getInstance()
    private val shooter: Shooter = Shooter.getInstance()
    private val conveyor: Conveyor = Conveyor.getInstance()
    private val hood: Hood = Hood.getInstance()
    private val gripper: Gripper = Gripper.getInstance()

    private fun init(): Command {
        val driveAndAdjust = swerveDrive.driveAndAdjust(
            { ScoreConstants.AMP_ROTATION },
            { -ControllerInputs.driverController().leftY },
            { -ControllerInputs.driverController().leftX },
            SwerveConstants.AMP_TURN_TOLERANCE,
            0.1,
            true
        )
        val setShooterVelocity = shooter.setVelocity(
            ScoreConstants.SHOOTER_TOP_AMP_VELOCITY, ScoreConstants.SHOOTER_BOTTOM_AMP_VELOCITY
        )
        val setHoodAngle = hood.setAngle(ScoreConstants.HOOD_AMP_ANGLE)

        return Commands.parallel(
            setShooterVelocity,
            conveyor.setVelocity(ScoreConstants.CONVEYOR_AMP_VELOCITY),
            setHoodAngle,
        )
    }

    private fun end(): Command {
        return gripper.feed().andThen(Commands.parallel(shooter.stop(), conveyor.stop(), gripper.stop()))
    }

    override fun execute(): Command {
        return init().withTimeout(0.3).andThen(end()).handleInterrupt(end())
    }
}