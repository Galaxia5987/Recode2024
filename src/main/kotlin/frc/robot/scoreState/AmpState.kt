package frc.robot.scoreState

import edu.wpi.first.wpilibj2.command.Command

class AmpState : ScoreState {
    private val swerveDrive: SwerveDrive = SwerveDrive.getInstance()
    private val shooter: Shooter = Shooter.getInstance()
    private val conveyor: Conveyor = Conveyor.getInstance()
    private val hood: Hood = Hood.getInstance()
    private val gripper: Gripper = Gripper.getInstance()

    private fun init(): Runnable {
        return Runnable {
            Commands.parallel(
                swerveDrive.driveAndAdjust(
                    ScoreConstants.AMP_ROTATION,
                    { -ControllerInputs.getDriverController().leftX },
                    { -ControllerInputs.getDriverController().leftY },
                    0.1,
                    false
                ), shooter.setVelocity(
                    ScoreConstants.SHOOTER_TOP_AMP_VELOCITY,
                    ScoreConstants.SHOOTER_BOTTOM_AMP_VELOCITY
                ), conveyor.setVelocity(ScoreConstants.CONVEYOR_AMP_VELOCITY)
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