package frc.robot.commandGroups

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import frc.robot.ControllerInputs
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.leds.LEDConstants
import frc.robot.subsystems.leds.LEDs

object IntakeCommands {
    private val intake = Intake.getInstance()
    private val gripper = Gripper.getInstance()

    fun stopIntake(): Command = Commands.parallel(
        intake.stop(), gripper.stop(),
        ControllerInputs.stopRumble()
            .andThen(Commands.waitSeconds(2.0))
            .andThen(
                ConditionalCommand(
                    LEDs.getInstance().setSolidMode(LEDConstants.SHOOT_STATE_COLOR),
                    LEDs.getInstance().setSolidMode(LEDConstants.AMP_STATE_COLOR),
                    { RobotContainer.getState()=="Speaker"})
            ))

    fun intake(): Command {
        return Commands.parallel(
            intake.intake(), gripper.setRollerPower(0.4)
        )
            .until { gripper.hasNote && gripper.useSensor }
            .andThen(
                Commands.parallel(
                    intake.stop(),
                    gripper.setRollerPower(0.0),
                    ControllerInputs.startRumble().onlyIf{Robot.isTeleop},
                    LEDs.getInstance().setSolidMode(LEDConstants.HAS_NOTE_COLOR)
                ))
            .withName("intake")
    }

    fun outtake(): Command {
        return Commands.parallel(
            intake.outtake(),
            gripper.setRollerPower(-0.7)
        ).withName("outtake")
    }
}
