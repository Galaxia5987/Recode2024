package frc.robot.commandgroups

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.intake.Intake

object IntakeCommands {
    private val intake = Intake.getInstance()
    private val gripper = Gripper.getInstance()

    private fun stop(): Command = Commands.parallel(gripper.stopGripper(), intake.stop())
    private fun intake(): Command =
        Commands.parallel(gripper.gripperIn(), intake.intakeIn()).until { gripper.hasNote() }.andThen(
            stop()
        )

    private fun outtake(): Command = Commands.parallel(gripper.gripperOut(), intake.intakeOut())
        .finallyDo(Runnable { stop() })
}