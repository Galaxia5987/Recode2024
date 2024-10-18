package frc.robot.commandGroups

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.ControllerInputs
import frc.robot.lib.finallyDo
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.leds.LEDs

object IntakeCommands {
    private val intake = Intake.getInstance()
    private val gripper = Gripper.getInstance()

    private fun stopIntake(): Command = Commands.parallel(intake.stop(), gripper.stop())

    fun intake(): Command {
        return Commands.parallel(
            intake.intake(), gripper.setRollerPower(0.4)
        )
            .until { gripper.hasNote }
            .andThen(
                Commands.parallel(
                    intake.stop(),
                    gripper.setRollerPower(0.0),
                    ControllerInputs.startRumble(),
                    LEDs.getInstance().intakeCommand()
                ))
            .finallyDo(stopIntake().alongWith(ControllerInputs.stopRumble()))
            .withName("intake")
    }

    fun outtake(): Command {
        return Commands.parallel(
            intake.outtake(),
            gripper.setRollerPower(-0.7)
        )
            .finallyDo(stopIntake())
            .withName("outtake")
    }
}
