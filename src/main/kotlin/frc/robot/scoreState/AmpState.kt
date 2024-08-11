package frc.robot.scoreState

import edu.wpi.first.wpilibj2.command.Command

class AmpState : ScoreState {
    override fun execute(): Command {
        return StartEndCommand(
            { init() },
            { end() },
            swerveDrive, shooter, conveyor, hood, gripper
        )
    }
}